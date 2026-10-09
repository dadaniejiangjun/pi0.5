from pathlib import Path
import pytest,json
from server.training.train_demonstrations import parser,run

def args(tmp_path):
 return parser().parse_args(['--dataset-path',str(tmp_path/'data'),'--base-params',str(tmp_path/'base'),'--experiment','test_5000','--expected-dataset-sha256','a'*64,'--report',str(tmp_path/'report.json'),'--train'])

def test_explicit_authorization_and_step_limit(tmp_path):
 a=args(tmp_path);a.train=False
 with pytest.raises(PermissionError):run(a)
 a.train=True;a.steps=50001
 with pytest.raises(ValueError):run(a)
 assert not a.report.exists()

def test_run_reuses_audit_and_records_result(tmp_path,monkeypatch):
 from server.training import proxy_multi_episode_pilot as pilot
 seen={}
 def fake(dataset,**kwargs):seen.update(kwargs);return {'checkpoint':'saved'}
 monkeypatch.setattr(pilot,'execute',fake)
 a=args(tmp_path);r=run(a)
 assert r['status']=='COMPLETE'
 assert seen['steps']==5000 and seen['allow_long_training'] is True
 assert seen['expected_dataset_sha256']=='a'*64
 assert json.loads(a.report.read_text())['result']['checkpoint']=='saved'
 with pytest.raises(FileExistsError):run(a)

def test_failure_keeps_diagnostic_report(tmp_path,monkeypatch):
 from server.training import proxy_multi_episode_pilot as pilot
 def failed(*a,**k):raise RuntimeError('specific training failure')
 monkeypatch.setattr(pilot,'execute',failed)
 a=args(tmp_path)
 with pytest.raises(RuntimeError):run(a)
 assert json.loads(a.report.read_text())['status']=='FAILED'
