# Vendored kernel

Source: https://github.com/sirfonzie/mtk3smp-rp2040

Commit: `2e8e6ad480c2d84a369b12995694b9bf5e9ddfb3`

Selected include/, kernel/, lib/libtk/ and licenses from the course-linked port. Upstream source is unchanged. Built in single-core mode with the project SDK startup adapter; upstream startup, vector table and board pin initialization are excluded. This combination needs board validation and does not inherit upstream hardware qualification.
