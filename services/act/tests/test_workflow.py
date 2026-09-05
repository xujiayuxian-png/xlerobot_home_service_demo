import argparse
import json
from pathlib import Path
import tempfile
import unittest

from services.act.workflow import (
    qualification_document,
    main,
    parser,
    train_command,
    write_json_atomic,
)
from tools.lib.verify_model_files import EXPECTED_STRUCTURE, _validate_manifest, verify


class WorkflowTest(unittest.TestCase):
    def resume_fixture(self, output, step=2, total=10):
        checkpoint = output / 'checkpoints' / f'{step:06d}'
        model = checkpoint / 'pretrained_model'
        state = checkpoint / 'training_state'
        model.mkdir(parents=True)
        state.mkdir()
        (model / 'train_config.json').write_text(json.dumps({
            'output_dir': str(output), 'steps': total, 'scheduler': None,
            'batch_size': 1, 'policy': {'type': 'act'},
        }))
        (state / 'training_step.json').write_text(json.dumps({'step': step}))
        for path in [model / 'config.json', model / 'model.safetensors', *[
            state / name for name in ('optimizer_state.safetensors',
                                     'optimizer_param_groups.json', 'rng_state.safetensors')
        ]]:
            path.touch()
        (output / 'checkpoints/last').symlink_to(checkpoint.name)
        return checkpoint

    def test_resume_uses_saved_config_without_replacing_training_settings(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            checkpoint = self.resume_fixture(output)
            args = parser().parse_args(['train', '--resume', '--output', str(output)])
            command = train_command(args)
            self.assertIn(f'--config_path={checkpoint}/pretrained_model/train_config.json', command)
            self.assertIn('--resume=true', command)
            self.assertIn('--steps=10', command)
            self.assertIn('--policy.push_to_hub=false', command)
            self.assertFalse(any(item.startswith(('--dataset.', '--batch_size=', '--policy.type=')) for item in command))
            args.steps = 12
            self.assertIn('--steps=12', train_command(args))

    def test_resume_rejects_missing_or_incomplete_checkpoint_and_finished_target(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            args = parser().parse_args(['train', '--resume', '--output', str(output)])
            with self.assertRaisesRegex(ValueError, 'resume requires a local checkpoint'):
                train_command(args)
            checkpoint = self.resume_fixture(output)
            args.steps = 2
            with self.assertRaisesRegex(ValueError, 'greater than saved step'):
                train_command(args)
            args.steps = 4
            (checkpoint / 'training_state/optimizer_state.safetensors').unlink()
            with self.assertRaisesRegex(ValueError, 'incomplete'):
                train_command(args)

    def test_resume_rejects_wrong_output_and_checkpoint_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            checkpoint = self.resume_fixture(output)
            args = parser().parse_args(['train', '--resume', '--output', str(output), '--steps', '3'])
            (output / 'checkpoints/000003').mkdir()
            with self.assertRaisesRegex(ValueError, 'overwrite'):
                train_command(args)
            args.steps = 4
            config = checkpoint / 'pretrained_model/train_config.json'
            saved = json.loads(config.read_text())
            saved['output_dir'] = str(output / 'different-run')
            config.write_text(json.dumps(saved))
            with self.assertRaisesRegex(ValueError, 'does not match'):
                train_command(args)

    def test_resume_rejects_fresh_run_overrides(self):
        with self.assertRaisesRegex(SystemExit, 'restores original training settings'):
            main(['train', '--resume', '--batch-size=2', '--dry-run'])

    def test_new_train_keeps_5000_step_default(self):
        args = parser().parse_args(['train'])
        self.assertIn('--steps=5000', train_command(args))

    def test_train_command_locks_deployed_wrist_only_contract(self):
        arguments = argparse.Namespace(
            dataset="local/data",
            repo_id="xujiayuxian-png/xlerobot-glue-stick-grasp-30",
            output="outputs/model",
            job_name="test",
            steps=5000,
            batch_size=8,
            chunk_size=100,
            resume=False,
        )
        command = train_command(arguments)
        self.assertIn(
            "--dataset.repo_id=xujiayuxian-png/xlerobot-glue-stick-grasp-30",
            command,
        )
        self.assertIn("--dataset.root=local/data", command)
        self.assertNotIn("--dataset.repo_id=local/data", command)
        inputs = next(
            value.split("=", 1)[1]
            for value in command
            if value.startswith("--policy.input_features=")
        )
        self.assertEqual(
            set(json.loads(inputs)),
            {"observation.state", "observation.images.wrist"},
        )
        for expected in (
            "--policy.type=act",
            "--policy.chunk_size=100",
            "--policy.n_action_steps=100",
            "--policy.dim_model=512",
            "--policy.dim_feedforward=3200",
            "--policy.n_encoder_layers=4",
            "--policy.n_decoder_layers=1",
            "--policy.n_heads=8",
            "--policy.latent_dim=32",
            "--policy.use_vae=true",
            "--policy.use_amp=false",
            "--save_freq=5000",
        ):
            self.assertIn(expected, command)

    def test_local_qualification_hashes_every_checkpoint_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            checkpoint = Path(temporary) / "pretrained_model"
            checkpoint.mkdir()
            (checkpoint / "config.json").write_text("{}\n", encoding="utf-8")
            (checkpoint / "model.safetensors").write_bytes(b"weights")
            nested = checkpoint / "processors"
            nested.mkdir()
            (nested / "preprocessor.json").write_text("{}\n", encoding="utf-8")
            manifest = checkpoint / "model-manifest.json"

            document = qualification_document(
                checkpoint, manifest, "cuda", EXPECTED_STRUCTURE
            )
            write_json_atomic(manifest, document)

            self.assertEqual(
                set(document["files"]),
                {
                    "config.json",
                    "model.safetensors",
                    "processors/preprocessor.json",
                },
            )
            self.assertEqual(verify(manifest, checkpoint), 3)
            (checkpoint / "model.safetensors").write_bytes(b"tampered")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                verify(manifest, checkpoint)

    def test_local_qualification_rejects_unlisted_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            checkpoint = Path(temporary)
            (checkpoint / "config.json").write_text("{}", encoding="utf-8")
            (checkpoint / "model.safetensors").write_bytes(b"weights")
            manifest = checkpoint / "model-manifest.json"
            write_json_atomic(
                manifest,
                qualification_document(
                    checkpoint, manifest, "cuda", EXPECTED_STRUCTURE
                ),
            )
            (checkpoint / "unqualified.bin").write_bytes(b"new")
            with self.assertRaisesRegex(ValueError, "inventory differs"):
                verify(manifest, checkpoint)

    def test_pending_public_manifest_cannot_authorize_serving(self):
        manifest = Path(
            "assets/models/xlerobot-act-local-grasp-v1.manifest.json"
        )
        document = json.loads(manifest.read_text(encoding="utf-8"))
        with self.assertRaisesRegex(ValueError, "immutable Hub revision"):
            _validate_manifest(document)


if __name__ == "__main__":
    unittest.main()
