"""Persistent, hardware-free state for the reference head calibration sweep."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml

from .sample_set import TransformSampleSet
from .solver import HEAD_MODEL


FRAMES = {'base': 'base_link', 'moving': 'head_tilt_link',
          'camera': 'head_camera_link', 'target': 'calibration_target'}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def load_poses(path):
    value = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(value, dict) or value.get('schema') != 'xlerobot_calibration_pose_set/v1' \
            or value.get('workflow') != 'head_camera' or value.get('units') != 'rad':
        raise ValueError('expected the reference head_camera pose set in radians')
    poses = value.get('poses')
    if not isinstance(poses, list) or len(poses) < 12:
        raise ValueError('head pose set requires at least 12 poses')
    for row in poses:
        if not isinstance(row, dict) or set(row) != {'pan', 'tilt'}:
            raise ValueError('each head pose must contain pan and tilt')
        for name, lower, upper in [('pan', -0.3, 0.3), ('tilt', 0.6, 1.0)]:
            number = row[name]
            if isinstance(number, bool) or not isinstance(number, (int, float)) \
                    or not lower <= number <= upper:
                raise ValueError(f'head {name} is outside the reference capture envelope')
    if len({(row['pan'], row['tilt']) for row in poses}) != len(poses):
        raise ValueError('head poses must be distinct')
    return poses, hashlib.sha256(Path(path).read_bytes()).hexdigest()


class HeadSession:
    """A saved sweep never resumes motion implicitly, and never erases samples."""

    workflow = 'head_camera'
    model = HEAD_MODEL
    frames = FRAMES
    schema = 'xlerobot_head_capture_progress/v1'

    def __init__(self, directory, unit, poses, pose_hash, *, servo_hash=''):
        self.directory = Path(directory)
        self.path = self.directory / 'progress.yaml'
        self.sample_path = self.directory / 'samples.yaml'
        self.unit = unit
        self.poses = poses
        self.pose_hash = pose_hash
        self.document = {
            'schema': self.schema, 'unit_id': unit,
            'pose_sha256': pose_hash, 'servo_sha256': servo_hash,
            'phase': 'IDLE', 'message': 'ready to start',
            'pose_index': -1, 'pose_states': ['pending'] * len(poses),
            'pose_messages': [''] * len(poses), 'sample_count': 0,
            'sample_sha256': '', 'sample_identity': '', 'pending_capture': None,
            'result_uri': '', 'quality_passed': False, 'metrics': {},
        }
        if self.path.exists():
            restored = yaml.safe_load(self.path.read_text(encoding='utf-8'))
            if not isinstance(restored, dict) or set(restored) != set(self.document):
                raise ValueError('invalid visual calibration progress schema')
            if restored['schema'] != self.document['schema'] or restored['unit_id'] != unit \
                    or restored['pose_sha256'] != pose_hash or restored['servo_sha256'] != servo_hash:
                raise ValueError('visual progress belongs to a different unit, pose set or predecessor servo calibration')
            states = restored['pose_states']
            if not isinstance(states, list) or len(states) != len(poses) \
                    or any(state not in {'pending', 'moving', 'waiting', 'captured', 'skipped'}
                           for state in states):
                raise ValueError('invalid saved visual pose states')
            self.document = restored
            self.reconcile()
            if restored['phase'] not in {'COMPLETED', 'ERROR', 'IDLE'}:
                self.pause('restored saved progress; press start to continue')
        else:
            self._sync_samples()

    def _samples(self):
        if not self.sample_path.exists():
            return None
        return TransformSampleSet.read(
            self.sample_path, expected_model=self.model,
            expected_calibration_id=self.unit, expected_frames=self.frames,
        ).to_dict()

    def _sync_samples(self):
        samples = self._samples()
        self.document['sample_count'] = len(samples['samples']) if samples else 0
        self.document['sample_sha256'] = (
            hashlib.sha256(self.sample_path.read_bytes()).hexdigest() if samples else '')
        self.document['sample_identity'] = digest(samples)

    def reconcile(self):
        """Recover the one capture committed just before a process interruption."""
        samples = self._samples()
        actual_hash = hashlib.sha256(self.sample_path.read_bytes()).hexdigest() if samples else ''
        if actual_hash == self.document['sample_sha256']:
            self.document['pending_capture'] = None
            return
        pending = self.document['pending_capture']
        if samples and pending and len(samples['samples']) == self.document['sample_count'] + 1:
            last = samples['samples'][-1]
            preceding = dict(samples, samples=samples['samples'][:-1])
            previous_identity = digest(
                None if not self.document['sample_sha256'] else preceding)
            if previous_identity == self.document['sample_identity'] \
                    and last['quality'].get('capture_job_id') == pending['job_id']:
                self.document['pose_states'][pending['pose_index']] = 'captured'
                self.document['pose_messages'][pending['pose_index']] = 'capture recovered after interruption'
                self.document['pending_capture'] = None
                self._sync_samples()
                self.save()
                return
        raise ValueError('visual samples changed outside this sweep; preserve files and start a fresh capture')

    def save(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name('.progress.yaml.tmp')
        temporary.write_text(yaml.safe_dump(self.document, sort_keys=False), encoding='utf-8')
        temporary.replace(self.path)

    def begin(self):
        if self.document['phase'] == 'COMPLETED':
            raise ValueError('this sweep is complete; use --fresh for a new capture')
        self.reconcile()
        for index, state in enumerate(self.document['pose_states']):
            if state != 'captured':
                self.document['pose_states'][index] = 'pending'
        self.document.update(phase='IDLE', message='starting remaining poses')
        self.save()
        return [index for index, state in enumerate(self.document['pose_states']) if state != 'captured']

    def phase(self, phase, message, index=None):
        self.document.update(phase=phase, message=message)
        if index is not None:
            self.document['pose_index'] = index
            if phase in {'MOVING', 'WAITING'}:
                self.document['pose_states'][index] = phase.lower()
        self.save()

    def prepare_capture(self, index):
        job = f'{self.unit}:{self.workflow}:{self.pose_hash[:12]}:pose:{index}'
        self.document['pending_capture'] = {'pose_index': index, 'job_id': job}
        self.phase('CAPTURING', 'saving synchronized target and moving-link pose', index)
        return job

    def captured(self, index):
        self.reconcile()
        if self.document['pose_states'][index] != 'captured':
            raise ValueError('capture response arrived without its matching saved sample')

    def skipped(self, index, message):
        self.document['pending_capture'] = None
        self.document['pose_states'][index] = 'skipped'
        self.document['pose_messages'][index] = message
        self.document['message'] = message
        self.save()

    def pause(self, message):
        for index, state in enumerate(self.document['pose_states']):
            if state in {'moving', 'waiting'}:
                self.document['pose_states'][index] = 'pending'
        self.phase('PAUSED', message)

    def complete(self, result_uri, metrics):
        self.document.update(result_uri=result_uri, metrics=metrics, quality_passed=True)
        self.phase('COMPLETED', 'quality passed; saved to draft, active calibration unchanged')
