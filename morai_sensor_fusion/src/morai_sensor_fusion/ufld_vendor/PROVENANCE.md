Upstream: https://github.com/cfzd/Ultra-Fast-Lane-Detection (MIT, cfzd 2020).
Imported from src_굳/kcity2026_stack/kcity2026/vendor/ufld/model. Exact upstream commit unknown.
Changes: relative backbone import only; inference architecture preserved.
Local source SHA256:
model.py f8afe43b885bc85bb3eb7594bc232c17bf4915893c2076c4add67e2d3c546261
backbone.py 127c0509d3e105863c380a8ba27630a92e069d0c4d3bbe7c9af5c6b191e15e60

Weights: team-provided ufld_res18_kasa_ep029.pth; upstream code license does not establish weight license. Training optimizer and unused auxiliary tensors removed, inference tensors unchanged.
