"""Default capture requires verified SVO; legacy save never claims a recording."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

launcher = Path(__file__).resolve().parents[1]/'launch/deployed_20261015/start_rtabmap_calib_ab.sh'
source = launcher.read_text(encoding='utf-8').split("<<'PY_SAVE_REPORT'\n",1)[1].split('\nPY_SAVE_REPORT',1)[0]


class SaveReportTests(unittest.TestCase):
    def report(self, profile, verified):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);exported=root/'archive';exported.mkdir()
            (exported/'export_validation.json').write_text(json.dumps({'publish_allowed':True}))
            (exported/'submaps.json').write_text(json.dumps({'complete':True}))
            if verified:
                raw=root/'raw_recording';raw.mkdir()
                (raw/'verification.json').write_text(json.dumps({'complete':True}))
            q=subprocess.run([sys.executable,'-B','-',str(root),str(exported),'map.ply',profile,'dual'],
                             input=source,text=True,capture_output=True)
            return q.returncode,json.loads((root/'save_report.json').read_text())

    def test_default_missing_recording_fails(self):
        code,report=self.report('dual',False)
        self.assertEqual(code,1);self.assertFalse(report['complete'])
        self.assertTrue(report['raw_recording_required'])

    def test_default_verified_recording_saves(self):
        code,report=self.report('dual',True)
        self.assertEqual(code,0);self.assertTrue(report['complete'])
        self.assertTrue(report['raw_recording']['complete'])

    def test_legacy_map_save_explicitly_has_no_recording(self):
        code,report=self.report('legacy',False)
        self.assertEqual(code,0);self.assertTrue(report['complete'])
        self.assertFalse(report['raw_recording_required'])
        self.assertFalse(report['raw_recording']['complete'])
        self.assertEqual(report['raw_recording']['status'],'not_supported_by_legacy_profile')


if __name__=='__main__':unittest.main()
