import cv2
import numpy as np
import pytest
from laptop.recorder.episode import EpisodeWriter, CHANNELS


@pytest.mark.parametrize('camera_times,expected', [([1,3,4,6],'FILE_INTEGRITY_PASS'),([1,2],'INCOMPLETE')])
def test_boundary_frames_warning_but_no_overlap_failure(tmp_path,camera_times,expected):
    writer=EpisodeWriter(tmp_path,source='operator_demonstration')
    for channel in CHANNELS:
        for stamp in (camera_times if channel in ('base','wrist') else [3,4]):
            writer.add(channel,stamp,{'raw':0})
    for role in ('base','wrist'):
        video=cv2.VideoWriter(str(writer.path/f'{role}.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(64,48))
        assert video.isOpened()
        for _ in camera_times:video.write(np.zeros((48,64,3),np.uint8))
        video.release()
    report=writer.close()
    assert report['status']==expected
    assert len(report['warnings'])==2
    assert report['training_eligible'] is False
