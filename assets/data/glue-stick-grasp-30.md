# xlerobot-glue-stick-grasp-30 dataset card

This is the planned public training dataset for
`xlerobot-act-local-grasp-v1`. It contains all 30 accepted demonstrations of
the reference right arm grasping one yellow glue stick. The release license is
CC BY 4.0 and the fixed repository ID is
`xujiayuxian-png/xlerobot-glue-stick-grasp-30`.

The large episode and video files are not part of this Git repository. The Hub
upload and immutable revision are still pending; `manifest.yaml` records that
state explicitly. The dataset does not contain shuttlecock demonstrations, so
the shuttlecock demo is qualitative out-of-distribution evidence only.

Each published episode must preserve the six-joint order, 30 Hz timing,
measured follower state, next-state action semantics, wrist video, source unit
identity, and accepted review record described by the collection tools. No
success-rate or general-object claim is attached to these 30 demonstrations.

## Local pre-publication privacy review

The local release candidate contains 30 episodes, 60 MP4 files, and 12,358
decoded frames. All videos decoded successfully. An OpenCV face-candidate scan
flagged 183 frames; clustering produced 24 groups, and human review of boxed
contact sheets found only glue-stick labels, grippers, and table textures, with
zero visible faces. QR detection flagged 436 frames but decoded none; review
identified product-label texture rather than readable codes.

OCR sampled five time points from every video (300 frames): 295 contained some
Chinese or English text, while the review regexes found no email, URL, IP,
credential, address, phone, or ID pattern. The same sensitive-pattern scan had
zero hits in JSON, logs, and NPZ string fields. Human review of six global
contact sheets found no active screen, address, or credential.

This is evidence that the local review passed, not a guarantee that automated
detection can prove the absence of every privacy issue. Upload still requires
the user's explicit confirmation.
