# Image credits

| file | author | license | source |
|---|---|---|---|
| `lecture01/beuchet-peephole.jpg` | Herzi Pinki | CC BY-SA 4.0 | https://commons.wikimedia.org/wiki/File:Beuchet_chair_Kinderuni_Wien_01.jpg |
| `lecture01/beuchet-side.jpg` | Herzi Pinki | CC BY-SA 4.0 | https://commons.wikimedia.org/wiki/File:Beuchet_chair_Kinderuni_Wien_02.jpg |

Both resized to at most 900 px. All other figures are drawn in code (`shared/widgets/`) or inline SVG.

| file | author | license | source |
|---|---|---|---|
| `lecture02/owl.jpg` | Charlie Marshall | CC BY 2.0 | https://commons.wikimedia.org/wiki/File:Owl_Face_(52897827598).jpg |
| `lecture02/rabbit.jpg` | caroline legg | CC BY 2.0 | https://commons.wikimedia.org/wiki/File:European_Rabbit_(50219547912)_(cropped).jpg |

Lecture 2 photos resized to at most 600 px and re-encoded; credited on the slide.

| file | author | license | source |
|---|---|---|---|
| `lecture03/canary_wharf_band.jpg` | Andreas Mischok | CC0 | https://polyhaven.com/a/canary_wharf |

Lecture 3 panorama: Poly Haven's tonemapped 8K equirectangular JPG, cropped to elevation +40° … −35°, resized to 4096 × 854 and re-encoded; the views of the `stitch` and `panoproj` widgets are rendered from it (`shared/widgets/panoenv.js`).

| file | author | license | source |
|---|---|---|---|
| `lecture03/drone_ortho.jpg` | Bayerische Vermessungsverwaltung (Landesamt für Digitalisierung, Breitband und Vermessung) | CC BY 4.0 | https://commons.wikimedia.org/wiki/File:DOP20_-_Bayerische_Vermessungsverwaltung_32647_5288.tif (DOP20, https://geodaten.bayern.de/opengeodata/) |

Lecture 3 drone map: Commons' 3840 px rendering of the 1 km DOP20 tile, cropped to 1920 × 1280 px (500 m × 333 m, 0.26 m/px) and re-encoded; the drone photos of the `dronemap` widget are rendered from it.

`lecture03/feat_a.jpg`, `feat_b.jpg`, `orb_fast.png`, `orb_brief.png`: cut from `canary_wharf_band.jpg` (CC0, above) for the "Detect, describe, match" and "Descriptors: ORB" slides (photo 2 shifted, scaled 5%, turned 4°, darkened); keypoints, descriptors and matches computed on them.

| file | author | license | source |
|---|---|---|---|
| `lecture05/lab-75.jpg`, `lab-83.jpg`, `lab-84.jpg` | MIMUW robot learning lab (Mateusz Wyszyński), own photos | course authors | — |
| `lecture05/pi0-robot.mp4`, `pi0-robot-poster.jpg` | the π0 action-chunk denoising animation from the 2025/26 lecture deck (Mateusz Wyszyński), converted from GIF to H.264 | course authors | — |
| `lecture05/drone_gates.mp4` (+ `.jpg`) | the drone homework's reference solution flying through the gates, rendered offscreen with MuJoCo (same model as below) | course authors | — |
| `lecture05/drone_dt001.mp4`, `drone_dt004.mp4` (+ `.jpg` posters) | the homework's Skydio X2 drone (MuJoCo model from mujoco_menagerie, Apache 2.0) under a P altitude controller, rendered offscreen with MuJoCo at two time steps for the lecture | course authors; model: Skydio / DeepMind | https://github.com/google-deepmind/mujoco_menagerie |
| `lecture05/data/flow_gauss_moons.json` | a small flow-matching network trained for the DNN lecture (`shared/widgets/flowlib.js`), used by the deformation widget | course authors | — |
| `lecture05/pi07-overview.mp4`, `pi07-overview-poster.jpg` | π0.7 overview demonstration, resized to 960×540 and with audio removed for lecture playback; poster extracted from the video | Physical Intelligence, 2026 | https://www.pi.website/blog/pi07 (video: https://website.pi-asset.com/pi07/Pi07VFINAL.mp4) |

Video thumbnails on the lecture 5 slides are loaded from YouTube (`img.youtube.com`) and link to the videos; they are not stored here.
