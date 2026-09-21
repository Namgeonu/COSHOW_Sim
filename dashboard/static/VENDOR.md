# Offline third-party assets

Acquired on 2026-09-13 from the official GitHub projects. All files below are unmodified upstream bytes, pinned to a stable release tag and its full Git commit. The acquisition verified each file against the upstream Git blob SHA-1; SHA-256 values below identify the checked-in copies.

## Versions and sources

- **Pretendard v1.3.9** — [official release](https://github.com/orioncactus/pretendard/releases/tag/v1.3.9), commit `5c41199ea0024a9e0b2cb31735265056e5472d76`. SIL Open Font License 1.1, including upstream copyright notices. This is the full Korean Pretendard Variable WOFF2, not a dynamic subset.
- **Three.js r186 / 0.186.0** — [official release](https://github.com/mrdoob/three.js/releases/tag/r186), commit `148ef33ecb6d2502ff796d4554abd1549c95d519`. MIT license. The official release was published on 2026-09-08; this stable tag is pinned rather than following a moving development branch.

## File inventory

| Local path (relative to `static/`) | Version | Bytes | SHA-256 | Official commit-pinned source |
| --- | --- | ---: | --- | --- |
| `fonts/PretendardVariable.woff2` | v1.3.9 | 2057688 | `9599f12fd42fc0bce1cd50b47a0c022e108d7aa64dd0d1bb0ed44f3282d900b4` | [upstream file](https://raw.githubusercontent.com/orioncactus/pretendard/5c41199ea0024a9e0b2cb31735265056e5472d76/packages/pretendard/dist/web/variable/woff2/PretendardVariable.woff2) |
| `fonts/Pretendard-OFL-1.1.txt` | v1.3.9 | 4418 | `d31ddd9f2bed32fd7e302a205cf2380ba0de6529152d239ef99cfb6f261bfc04` | [upstream file](https://raw.githubusercontent.com/orioncactus/pretendard/5c41199ea0024a9e0b2cb31735265056e5472d76/LICENSE) |
| `vendor/three.module.js` | r186 | 662772 | `9052042d676cb0fdc1ddfefe193053f34b7ac0513a616fdac4535d49987812ea` | [upstream file](https://raw.githubusercontent.com/mrdoob/three.js/148ef33ecb6d2502ff796d4554abd1549c95d519/build/three.module.js) |
| `vendor/three.core.js` | r186 | 1458113 | `9edde002b066a9a05676a6127f67735b62baf399bdea529f2f7e31657da769e6` | [upstream file](https://raw.githubusercontent.com/mrdoob/three.js/148ef33ecb6d2502ff796d4554abd1549c95d519/build/three.core.js) |
| `vendor/Three-MIT.txt` | r186 | 1081 | `8b378ebe60e2fe500158cb0ac71cb5e8b7d92953c2abcc63a0eb90499653b5bc` | [upstream file](https://raw.githubusercontent.com/mrdoob/three.js/148ef33ecb6d2502ff796d4554abd1549c95d519/LICENSE) |

## Local loading contract

- `css/glass.css` registers the local font as `Pretendard`, with `font-weight: 45 920` and `font-style: normal`, at `/static/fonts/PretendardVariable.woff2`. The [upstream CSS](https://raw.githubusercontent.com/orioncactus/pretendard/5c41199ea0024a9e0b2cb31735265056e5472d76/packages/pretendard/dist/web/variable/pretendardvariable.css) documents this axis range; its CSS file is not loaded at runtime.
- Import `../vendor/three.module.js` from a module in `static/js/`. `three.module.js` imports and re-exports `./three.core.js`; both files must ship together. `three.core.js` has no static module imports. No CDN or import map is required.
- M3 checks local module import and offline font loading. The actual Three.js scene is implemented in M4. Three.js asset loaders are only invoked when the application calls them; any later model, image, or texture paths must also be local.
- Preserve `fonts/Pretendard-OFL-1.1.txt` and `vendor/Three-MIT.txt` with the files when distributing the dashboard.

## Updating

Choose an explicit stable upstream tag, resolve its full commit, retrieve these same source files and any new relative module dependencies, preserve the corresponding license texts, and update this inventory after recomputing file sizes and SHA-256. Repeat the local import and browser offline checks before accepting an update.

Acquisition and static dependency evidence: `../REPORTS/evidence/M3_assets.log`.
