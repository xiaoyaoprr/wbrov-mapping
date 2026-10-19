import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).parents[1]/'runtime/libexec/wbrov_corner_recovery'))
import measured_bridge as m
import reference_policy

class TagDepthCandidateTests(unittest.TestCase):
    def setUp(self):
        data=json.loads((Path(__file__).parent/'fixtures/tag13_depth_candidate.json').read_text())
        rows=[m.observation(r['binding'],r['body']) for r in data['rows']]
        self.old=m.best_old_triplet([r for r in rows if r['epoch']==0])
        self.current=sorted((r for r in rows if r['epoch']==1),key=lambda r:r['stamp'])
        self.assertIsNotNone(self.old)

    def test_recorded_pnp_rejection_reaches_depth_as_candidate_only(self):
        strict,reason=m.propose(self.old,self.current)
        self.assertIsNone(strict)
        self.assertEqual(reason,'three_view_alignment_disagreement_or_baseline')
        candidate,reason=m.propose_depth_candidate(self.old,self.current)
        self.assertIsNotNone(candidate)
        self.assertTrue(candidate['requires_core_depth_verification'])
        self.assertFalse(candidate['pnp_alignment_certified'])
        self.assertGreater(candidate['alignment_consistency']['max_tilt_deg'],2.)
        self.assertEqual(candidate['current_ids'],[110,111,114])
        self.assertEqual(len({i for e in candidate['evidence'] for i in (e['from_id'],e['to_id'])}),6)
        self.assertNotIn('registered',candidate)

    def test_real_reference_selection_uses_depth_candidate(self):
        calls=[]
        result=reference_policy.choose_candidate([dict(epoch=0,reference=self.old)],self.current,
            lambda *args:calls.append(args))
        self.assertTrue(result['candidate_only'])
        self.assertEqual(result['reference_source_epoch'],0)
        self.assertEqual(calls[-1][-1],'three_independent_tag_views_for_core_depth_verification')

    def test_same_view_does_not_create_independent_evidence(self):
        for row in self.current:row['body']=self.current[0]['body'].copy()
        self.assertIsNone(m.propose_depth_candidate(self.old,self.current)[0])

    def test_stale_or_repeated_measurements_rejected(self):
        for mutate in ('stale','repeat_id','repeat_stamp'):
            current=copy.deepcopy(self.current)
            if mutate=='stale':current[0]['stamp']=current[-1]['stamp']-8000000001
            elif mutate=='repeat_id':current[1]['id']=current[0]['id']
            else:current[1]['stamp']=current[0]['stamp']
            with self.subTest(mutate=mutate):self.assertIsNone(m.propose_depth_candidate(self.old,current)[0])

    def test_mixed_tags_and_epochs_rejected(self):
        for key in ('tag_id','epoch'):
            current=copy.deepcopy(self.current);current[0][key]+=1
            with self.subTest(key=key):self.assertIsNone(m.propose_depth_candidate(self.old,current)[0])

if __name__=='__main__':unittest.main()
