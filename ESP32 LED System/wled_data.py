"""
Index-ordered WLED effect and palette names.

These name lists are NOT retrievable over the serial JSON link (WLED only serves
them via the HTTP endpoints /json/eff and /json/pal), so they are hardcoded here.
The list *position* is the integer that gets sent to the device as the segment
"fx" (effect) or "pal" (palette) value, so ORDER MATTERS — reserved/removed slots
are kept as "RSVD" to preserve index alignment.

Source: WLED firmware ~0.14.x (JSON_mode_names / JSON_palette_names).
If the connected device reports a different major/minor version (info.ver), the
indices may have shifted; the UI logs a warning when info.ver does not start with
EFFECTS_SOURCE_VERSION.
"""

EFFECTS_SOURCE_VERSION = "0.14"

# Effect names, index == fx value. "RSVD" entries are reserved/removed slots that
# must stay in place to keep every following index correct.
EFFECTS = [
    "Solid",                # 0
    "Blink",                # 1
    "Breathe",              # 2
    "Wipe",                 # 3
    "Wipe Random",          # 4
    "Random Colors",        # 5
    "Sweep",                # 6
    "Dynamic",              # 7
    "Colorloop",            # 8
    "Rainbow",              # 9
    "Scan",                 # 10
    "Scan Dual",            # 11
    "Fade",                 # 12
    "Theater",              # 13
    "Theater Rainbow",      # 14
    "Running",              # 15
    "Saw",                  # 16
    "Twinkle",              # 17
    "Dissolve",             # 18
    "Dissolve Rnd",         # 19
    "Sparkle",              # 20
    "Sparkle Dark",         # 21
    "Sparkle+",             # 22
    "Strobe",               # 23
    "Strobe Rainbow",       # 24
    "Strobe Mega",          # 25
    "Blink Rainbow",        # 26
    "Android",              # 27
    "Chase",                # 28
    "Chase Random",         # 29
    "Chase Rainbow",        # 30
    "Chase Flash",          # 31
    "Chase Flash Rnd",      # 32
    "Rainbow Runner",       # 33
    "Colorful",             # 34
    "Traffic Light",        # 35
    "Sweep Random",         # 36
    "Chase 2",              # 37
    "Aurora",               # 38
    "Stream",               # 39
    "Scanner",              # 40
    "Lighthouse",           # 41
    "Fireworks",            # 42
    "Rain",                 # 43
    "Tetrix",               # 44
    "Fire Flicker",         # 45
    "Gradient",             # 46
    "Loading",              # 47
    "RSVD",                 # 48
    "Fairy",                # 49
    "Two Dots",             # 50
    "Fairytwinkle",         # 51
    "Running Dual",         # 52
    "RSVD",                 # 53
    "Chase 3",              # 54
    "Tri Wipe",             # 55
    "Tri Fade",             # 56
    "Lightning",            # 57
    "ICU",                  # 58
    "Multi Comet",          # 59
    "Scanner Dual",         # 60
    "Stream 2",             # 61
    "Oscillate",            # 62
    "Pride 2015",           # 63
    "Juggle",               # 64
    "Palette",              # 65
    "Fire 2012",            # 66
    "Colorwaves",           # 67
    "Bpm",                  # 68
    "Fill Noise",           # 69
    "Noise 1",              # 70
    "Noise 2",              # 71
    "Noise 3",              # 72
    "Noise 4",              # 73
    "Colortwinkles",        # 74
    "Lake",                 # 75
    "Meteor",               # 76
    "Meteor Smooth",        # 77
    "Railway",              # 78
    "Ripple",               # 79
    "Twinklefox",           # 80
    "Twinklecat",           # 81
    "Halloween Eyes",       # 82
    "Solid Pattern",        # 83
    "Solid Pattern Tri",    # 84
    "Spots",                # 85
    "Spots Fade",           # 86
    "Glitter",              # 87
    "Candle",               # 88
    "Fireworks Starburst",  # 89
    "Fireworks 1D",         # 90
    "Bouncing Balls",       # 91
    "Sinelon",              # 92
    "Sinelon Dual",         # 93
    "Sinelon Rainbow",      # 94
    "Popcorn",              # 95
    "Drip",                 # 96
    "Plasma",               # 97
    "Percent",              # 98
    "Ripple Rainbow",       # 99
    "Heartbeat",            # 100
    "Pacifica",             # 101
    "Candle Multi",         # 102
    "Solid Glitter",        # 103
    "Sunrise",              # 104
    "Phased",               # 105
    "Twinkleup",            # 106
    "Noise Pal",            # 107
    "Sine",                 # 108
    "Phased Noise",         # 109
    "Flow",                 # 110
    "Chunchun",             # 111
    "Dancing Shadows",      # 112
    "Washing Machine",      # 113
    "RSVD",                 # 114
    "Blends",               # 115
    "TV Simulator",         # 116
    "Dynamic Smooth",       # 117
    "Spaceships",           # 118
    "Crazy Bees",           # 119
    "Ghost Rider",          # 120
    "Blobs",                # 121
    "Scrolling Text",       # 122
    "Drift Rose",           # 123
    "Distortion Waves",     # 124
    "Soap",                 # 125
    "Octopus",              # 126
    "Waving Cell",          # 127
    "Pixels",               # 128
    "Pixelwave",            # 129
    "Juggles",              # 130
    "Matripix",             # 131
    "Gravimeter",           # 132
    "Plasmoid",             # 133
    "Puddles",              # 134
    "Midnoise",             # 135
    "Noisemeter",           # 136
    "Freqwave",             # 137
    "Freqmatrix",           # 138
    "GEQ",                  # 139
    "Waterfall",            # 140
    "Freqpixels",           # 141
    "RSVD",                 # 142
    "Noisefire",            # 143
    "Puddlepeak",           # 144
    "Noisemove",            # 145
    "Noise2D",              # 146
    "Perlin Move",          # 147
    "Ripple Peak",          # 148
    "Firenoise",            # 149
    "Squared Swirl",        # 150
    "RSVD",                 # 151
    "DNA",                  # 152
    "Matrix",               # 153
    "Metaballs",            # 154
    "Freqmap",              # 155
    "Gravcenter",           # 156
    "Gravcentric",          # 157
    "Gravfreq",             # 158
    "DJ Light",             # 159
    "Funky Plank",          # 160
    "RSVD",                 # 161
    "Pulser",               # 162
    "Blurz",                # 163
    "Drift",                # 164
    "Waverly",              # 165
    "Sun Radiation",        # 166
    "Colored Bursts",       # 167
    "Julia",                # 168
    "RSVD",                 # 169
    "RSVD",                 # 170
    "RSVD",                 # 171
    "Game Of Life",         # 172
    "Tartan",               # 173
    "Polar Lights",         # 174
    "Swirl",                # 175
    "Lissajous",            # 176
    "Frizzles",             # 177
    "Plasma Ball",          # 178
    "Flow Stripe",          # 179
    "Hiphotic",             # 180
    "Sindots",              # 181
    "DNA Spiral",           # 182
    "Black Hole",           # 183
    "Wavesins",             # 184
    "Rocktaves",            # 185
    "Akemi",                # 186
]

# Palette names, index == pal value.
PALETTES = [
    "Default",              # 0
    "Random Cycle",         # 1
    "Color 1",              # 2
    "Colors 1&2",           # 3
    "Color Gradient",       # 4
    "Colors Only",          # 5
    "Party",                # 6
    "Cloud",                # 7
    "Lava",                 # 8
    "Ocean",                # 9
    "Forest",               # 10
    "Rainbow",              # 11
    "Rainbow Bands",        # 12
    "Sunset",               # 13
    "Rivendell",            # 14
    "Breeze",               # 15
    "Red & Blue",           # 16
    "Yellowout",            # 17
    "Analogous",            # 18
    "Splash",               # 19
    "Pastel",               # 20
    "Sunset 2",             # 21
    "Beach",                # 22
    "Vintage",              # 23
    "Departure",            # 24
    "Landscape",            # 25
    "Beech",                # 26
    "Sherbet",              # 27
    "Hult",                 # 28
    "Hult 64",              # 29
    "Drywet",               # 30
    "Jul",                  # 31
    "Grintage",             # 32
    "Rewhi",                # 33
    "Tertiary",             # 34
    "Fire",                 # 35
    "Icefire",              # 36
    "Cyane",                # 37
    "Light Pink",           # 38
    "Autumn",               # 39
    "Magenta",              # 40
    "Magred",               # 41
    "Yelmag",               # 42
    "Yelblu",               # 43
    "Orange & Teal",        # 44
    "Tiamat",               # 45
    "April Night",          # 46
    "Orangery",             # 47
    "C9",                   # 48
    "Sakura",               # 49
    "Aurora",               # 50
    "Atlantica",            # 51
    "C9 2",                 # 52
    "C9 New",               # 53
    "Temperature",          # 54
    "Aurora 2",             # 55
    "Retro Clown",          # 56
    "Candy",                # 57
    "Toxy Reaf",            # 58
    "Fairy Reaf",           # 59
    "Semi Blue",            # 60
    "Pink Candy",           # 61
    "Red Reaf",             # 62
    "Aqua Flash",           # 63
    "Yelblu Hot",           # 64
    "Lite Light",           # 65
    "Red Flash",            # 66
    "Blink Red",            # 67
    "Red Shift",            # 68
    "Red Tide",             # 69
    "Candy2",               # 70
]
