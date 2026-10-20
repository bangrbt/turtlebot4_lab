#!/usr/bin/env python3
"""Check PDF text bounds and TeX diagnostics; complement with visual inspection."""
import argparse,json,re,subprocess,tempfile,xml.etree.ElementTree as ET
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('--pdf',type=Path,default=Path(__file__).with_name('report.pdf'));p.add_argument('--log',type=Path,default=Path(__file__).with_name('report.log'));p.add_argument('--output',type=Path,default=Path(__file__).with_name('layout_checks.json'));a=p.parse_args()
 with tempfile.TemporaryDirectory(prefix='lab05-layout-') as tmp:
  out=Path(tmp)/'words.html';subprocess.run(['pdftotext','-bbox-layout',str(a.pdf),str(out)],check=True)
  # Poppler may emit raw C0 controls for math glyphs; remove invalid XML chars.
  raw=re.sub('[\x00-\x08\x0b\x0c\x0e-\x1f]','',out.read_text())
  ns={'h':'http://www.w3.org/1999/xhtml'};tree=ET.fromstring(raw);pages=tree.findall('.//h:page',ns);outside=[]
  for i,page in enumerate(pages,1):
   for word in page.findall('.//h:word',ns):
    box=[float(word.attrib[k]) for k in ['xMin','yMin','xMax','yMax']];x0,y0,x1,y1=box
    if x0<45 or x1>560 or y0<24 or y1>820:outside.append(dict(page=i,text=word.text,bbox=box))
 log=a.log.read_text()
 checks=dict(pages=len(pages),overfull_boxes=log.count('Overfull'),undefined_references=log.count('undefined references'),missing_characters=log.count('Missing character'),words_outside_safe_page_bounds=outside,safe_bounds_points=[45,24,560,820],scope='Text bounds include header/footer. TeX diagnostics and bounds complement visual inspection of diagrams/tables/screenshots.')
 a.output.write_text(json.dumps(checks,ensure_ascii=False,indent=2)+'\n');print(json.dumps(checks,ensure_ascii=False,indent=2))
 return int(any([checks['overfull_boxes'],checks['undefined_references'],checks['missing_characters'],outside]))
if __name__=='__main__':raise SystemExit(main())
