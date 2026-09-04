# Source installation

## Supported computers

The robot and GPU are separate Ubuntu 24.04 x86_64 machines. The robot needs a
normal ROS 2 Jazzy installation. The verified GPU is an RTX 3080 exposed to
Ubuntu 24.04 in WSL2. Other operating systems, ROS distributions, and GPU
stacks are not release targets for this repository.

Clone recursively on both machines, copy the two ignored local templates, and
edit them for your LAN and files:

```bash
git clone --recurse-submodules <repository-url>
cd xlerobot_home_service_demo
cp config/local.example.yaml config/local.yaml
cp .env.example .env
```

Generate a different ACT token for your installation and put the same token in
the robot and GPU `.env` files. Never expose ports 8765 or 8766 to the public
Internet.

## GPU computer

Install Miniconda first. `tools/setup gpu` creates two source environments
because their tested Python, NumPy, and PyTorch versions conflict:

- `.xlerobot/venvs/classical`: Python 3.10.20 and the classical service pins.
- `.xlerobot/venvs/act`: Python 3.12.13 and the ACT service pins.

```bash
./tools/setup gpu
```

This prepares the default ACT and classical-centroid routes. To compile the
optional pinned native GPD candidate generator and its system dependencies,
run `./tools/setup gpu --with-gpd` instead.

Setup explicitly downloads and verifies the pinned SAM 2 snapshot. Start LM
Studio `0.4.12+1` separately and load the manifest-recorded Q4_K_M artifact as
`qwen/qwen3-vl-4b`. The API doctor can check the served name; compare the local
GGUF revision and hashes with `assets/models/manifest.yaml`. Put the verified ACT checkpoint
at the repo-relative `models.act_checkpoint` and its qualification record at
`models.act_manifest`. Before the initial Hub release, these must be copied
together from the vetted local result; see `act-workflow.md` to produce one.
After an immutable Hub revision is published, `tools/act download` installs and
hash-checks both. Inspect `../../assets/models/manifest.yaml` first; no weight
is committed to Git.

## Robot computer

The setup command runs rosdep, creates `.venv/robot`, builds the web frontend,
and builds the ROS workspace from source:

```bash
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

The three optional flags are explicit for licensing or provenance reasons. The person detector
installs the AGPL-3.0 Ultralytics extra, downloads `yolov8n.pt` from its recorded
provider URL, and verifies its size and SHA-256. The KWS flag warns that the
upstream model/word-list terms remain unresolved, then downloads the exact
release directly from its provider; voice mode requires it, and this repository
does not mirror it. Voice generation calls Edge TTS and
writes ignored local MP3 files; the historical MP3 files are not distributed.

Setup always downloads the MIT-licensed, manifest-pinned Whisper snapshot and
checks every runtime file's SHA-256. It downloads and checks KWS only with
`--with-kws-model`. This is separate from `--generate-voice-prompts`, which
only creates local spoken-response MP3 files.

Re-run `tools/setup` after a dependency or frontend change. The resolved Python
versions are recorded locally under `.xlerobot/environment/`, which is ignored.

## Configuration ownership

`config/local.yaml` contains machine paths, LAN URLs, device aliases, and demo
defaults. `.env` contains only secrets. Algorithm and ROS controller parameters
remain inside their owning packages. Maps, named-place files, calibration,
models, and recordings stay outside source control.
