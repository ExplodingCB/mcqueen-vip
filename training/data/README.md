# training/data

Source catalog and ingest tooling for perception training footage. The plan and the source survey are in [docs/08-training-data.md](../../docs/08-training-data.md).

```
pip install -r requirements.txt
python fetch_youtube.py --dry-run          # measure hours per source, download nothing
python fetch_youtube.py                    # download Creative Commons videos only
python fetch_youtube.py --stats            # manifest summary
python extract_frames.py                   # 2 fps, dedupe, frames/index.csv
pytest -q                                  # filters, yaml sanity, synthetic extraction
```

`sources.yaml` is the only file to edit when adding a channel, playlist or query. Set `permission: granted` on a source only after written permission is on file; until then only Creative Commons videos from it are downloaded. `raw/`, `frames/` and `manifest.jsonl` are git-ignored: footage never enters the repository.
