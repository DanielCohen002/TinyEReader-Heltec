
(function () {
  'use strict';

  function normalizePath(path) {
    var parts = path.split('/');
    var out = [];
    for (var i = 0; i < parts.length; i++) {
      var part = parts[i];
      if (part === '' || part === '.') continue;
      if (part === '..') out.pop();
      else out.push(part);
    }
    return out.join('/');
  }

  function readZip(buf) {
    var view = new DataView(buf);
    var bytes = new Uint8Array(buf);
    var eocdSig = 0x06054b50;
    var pos = -1;
    var minPos = Math.max(0, bytes.length - 65557);
    for (var i = bytes.length - 22; i >= minPos; i--) {
      if (view.getUint32(i, true) === eocdSig) { pos = i; break; }
    }
    if (pos === -1) throw new Error("Not a valid EPUB (zip end-of-directory record not found).");

    var cdEntries = view.getUint16(pos + 10, true);
    var cdOffset = view.getUint32(pos + 16, true);

    var entries = {};
    var offset = cdOffset;
    for (var e = 0; e < cdEntries; e++) {
      var sig = view.getUint32(offset, true);
      if (sig !== 0x02014b50) throw new Error("Corrupt EPUB (bad zip central directory entry).");
      var method = view.getUint16(offset + 10, true);
      var compSize = view.getUint32(offset + 20, true);
      var nameLen = view.getUint16(offset + 28, true);
      var extraLen = view.getUint16(offset + 30, true);
      var commentLen = view.getUint16(offset + 32, true);
      var localOffset = view.getUint32(offset + 42, true);
      var name = new TextDecoder('utf-8').decode(bytes.subarray(offset + 46, offset + 46 + nameLen));
      entries[name] = { method: method, compSize: compSize, localOffset: localOffset };
      offset += 46 + nameLen + extraLen + commentLen;
    }

    async function readEntry(name) {
      var meta = entries[name];
      if (!meta) throw new Error('Missing "' + name + '" in EPUB.');
      var lo = meta.localOffset;
      var lNameLen = view.getUint16(lo + 26, true);
      var lExtraLen = view.getUint16(lo + 28, true);
      var dataStart = lo + 30 + lNameLen + lExtraLen;
      var compData = bytes.subarray(dataStart, dataStart + meta.compSize);
      if (meta.method === 0) return compData;
      if (meta.method === 8) {
        if (typeof DecompressionStream === 'undefined') {
          throw new Error("This browser can't unzip EPUBs (no DecompressionStream support) -- try a newer version of Chrome, Edge, Firefox, or Safari, or use tools/epub_to_txt.py instead.");
        }
        var stream = new Blob([compData]).stream().pipeThrough(new DecompressionStream('deflate-raw'));
        var out = await new Response(stream).arrayBuffer();
        return new Uint8Array(out);
      }
      throw new Error('Unsupported zip compression method ' + meta.method + ' for "' + name + '".');
    }

    return { readEntry: readEntry };
  }

  function getByLocalName(doc, localName) {
    var all = doc.getElementsByTagName('*');
    var out = [];
    for (var i = 0; i < all.length; i++) {
      if (all[i].localName === localName) out.push(all[i]);
    }
    return out;
  }

  var HEADING_TAGS = { H1: true, H2: true, H3: true };
  var SKIP_TAGS = { SCRIPT: true, STYLE: true, HEAD: true };
  var BLOCK_TAGS = { P: true, DIV: true, LI: true, TR: true, BLOCKQUOTE: true, BR: true };

  function extractChapterAwareText(html) {
    var doc = new DOMParser().parseFromString(html, 'text/html');
    var chunks = [];
    var headingOffsets = [];
    var length = 0;
    var atLineStart = true;

    function emit(text) {
      if (!text) return;
      chunks.push(text);
      length += text.length;
      atLineStart = text.charAt(text.length - 1) === '\n';
    }
    function breakParagraph() {
      if (!atLineStart) emit('\n\n');
    }
    function walk(node) {
      if (node.nodeType === 3) {  // TEXT_NODE
        var text = node.nodeValue.replace(/[ \t\r\f\v]+/g, ' ');
        if (text.trim() === '') return;
        if (atLineStart) text = text.replace(/^ +/, '');
        emit(text);
        return;
      }
      if (node.nodeType !== 1) return;  // ELEMENT_NODE
      var tag = node.tagName;
      if (SKIP_TAGS[tag]) return;
      var isHeading = HEADING_TAGS[tag];
      var isBlock = BLOCK_TAGS[tag];
      if (isHeading) {
        breakParagraph();
        headingOffsets.push(length);
      } else if (isBlock) {
        breakParagraph();
      }
      for (var c = 0; c < node.childNodes.length; c++) walk(node.childNodes[c]);
      if (isHeading || isBlock) breakParagraph();
    }
    walk(doc.body || doc.documentElement);
    return { text: chunks.join('').trim(), headingOffsets: headingOffsets };
  }

  async function epubToTxt(arrayBuffer) {
    var zip = readZip(arrayBuffer);

    var containerBytes = await zip.readEntry('META-INF/container.xml');
    var containerXml = new DOMParser().parseFromString(new TextDecoder('utf-8').decode(containerBytes), 'application/xml');
    var rootfiles = getByLocalName(containerXml, 'rootfile');
    if (rootfiles.length === 0) throw new Error("Could not find the EPUB's content file (missing container.xml rootfile).");
    var opfPath = rootfiles[0].getAttribute('full-path');
    var opfDir = opfPath.indexOf('/') === -1 ? '' : opfPath.substring(0, opfPath.lastIndexOf('/'));

    var opfBytes = await zip.readEntry(opfPath);
    var opfXml = new DOMParser().parseFromString(new TextDecoder('utf-8').decode(opfBytes), 'application/xml');

    var manifest = {};
    getByLocalName(opfXml, 'item').forEach(function (item) {
      var href = item.getAttribute('href');
      try { href = decodeURIComponent(href); } catch (e) {}
      manifest[item.getAttribute('id')] = href;
    });

    var spinePaths = [];
    getByLocalName(opfXml, 'itemref').forEach(function (itemref) {
      var href = manifest[itemref.getAttribute('idref')];
      if (!href) return;
      var path = opfDir ? opfDir + '/' + href : href;
      spinePaths.push(normalizePath(path));
    });
    if (spinePaths.length === 0) throw new Error('Could not find any readable content in this EPUB.');

    var fileTexts = [];
    var fileHeadingOffsets = [];
    for (var i = 0; i < spinePaths.length; i++) {
      var bytes;
      try {
        bytes = await zip.readEntry(spinePaths[i]);
      } catch (err) {
        continue;
      }
      var extracted = extractChapterAwareText(new TextDecoder('utf-8').decode(bytes));
      if (extracted.text) {
        fileTexts.push(extracted.text);
        fileHeadingOffsets.push(extracted.headingOffsets);
      }
    }
    if (fileTexts.length === 0) throw new Error('Could not extract any text from this EPUB.');

    var totalHeadings = fileHeadingOffsets.reduce(function (sum, offs) { return sum + offs.length; }, 0);
    var pieces = [];
    if (totalHeadings >= 2) {
      var isFirstHeadingOverall = true;
      for (var f = 0; f < fileTexts.length; f++) {
        var text = fileTexts[f];
        var offsets = fileHeadingOffsets[f];
        var cursor = 0;
        for (var h = 0; h < offsets.length; h++) {
          var off = offsets[h];
          if (isFirstHeadingOverall) { isFirstHeadingOverall = false; continue; }
          pieces.push(text.substring(cursor, off));
          pieces.push('\f');
          cursor = off;
        }
        pieces.push(text.substring(cursor));
        pieces.push('\n\n');
      }
    } else {
      for (var j = 0; j < fileTexts.length; j++) {
        if (j > 0) pieces.push('\f');
        pieces.push(fileTexts[j]);
        pieces.push('\n\n');
      }
    }
    return pieces.join('').trim() + '\n';
  }

  window.epubToTxt = epubToTxt;
})();
