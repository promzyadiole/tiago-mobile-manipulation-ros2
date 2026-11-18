#!/usr/bin/env python3
"""Figures of the link attacher experiment (section 15.1).

Run:  python3 make_attach_figures.py
Input:  images/attach/*.png and attach_experiment.json (from attach_experiment.py)
Output: images/fig_attach_photos.png, images/fig_attach_heights.png
"""
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, 'images')
SRC = os.path.join(IMG, 'attach')

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                     'axes.edgecolor': '#555555', 'axes.titleweight': 'bold'})

CASES = [('A', 'BEFORE: no attacher, fingers touch the bottle', '#c0392b'),
         ('B', 'BEFORE: no attacher, fingers squeeze 5 mm', '#d35400'),
         ('C', 'AFTER: link attacher (fingers 4 mm short)', '#1e8449')]
STAGES = [('1_ready', '1. bottle between the open fingers'),
          ('2_closed', '2. fingers closed'),
          ('3_lifted', '3. gripper lifted 12 cm')]


def fig_photos():
    fig, axes = plt.subplots(3, 3, figsize=(9.0, 7.4))
    for r, (case, title, colour) in enumerate(CASES):
        for c, (stage, label) in enumerate(STAGES):
            ax = axes[r, c]
            im = Image.open(os.path.join(SRC, f'{case}{stage}.png'))
            w, h = im.size
            ax.imshow(im.crop((0, 0, w, int(h * 0.92))))
            ax.set_xticks([])
            ax.set_yticks([])
            for s in ax.spines.values():
                s.set_edgecolor(colour)
                s.set_linewidth(2.2)
            if r == 0:
                ax.set_title(label, fontsize=9)
        axes[r, 0].set_ylabel(f'{case}', rotation=0, fontsize=13, fontweight='bold',
                              color=colour, labelpad=12, va='center')
        axes[r, 1].text(0.5, -0.06, title, transform=axes[r, 1].transAxes, ha='center',
                        va='top', fontsize=9, fontweight='bold', color=colour)
    fig.subplots_adjust(left=0.04, right=0.995, top=0.96, bottom=0.03, wspace=0.03, hspace=0.16)
    fig.savefig(os.path.join(IMG, 'fig_attach_photos.png'), dpi=160)
    plt.close(fig)


def fig_heights():
    d = json.load(open(os.path.join(SRC, 'attach_experiment.json')))
    z0 = d['results']['bottle_start_z']
    fig, axes = plt.subplots(1, 3, figsize=(9.0, 2.9), sharey=True)
    for ax, (case, title, colour) in zip(axes, CASES):
        s = [v for v in d['samples'] if v['case'] == case]
        t = [v['t'] for v in s]
        g = [(v['gz'] - s[0]['gz']) * 100 for v in s]
        b = [(v['bz'] - z0) * 100 for v in s]
        ax.plot(t, g, color='#555555', lw=1.6, ls='--', label='gripper')
        ax.plot(t, b, color=colour, lw=2.2, label='bottle')
        tilt = max(v['tilt'] for v in s)
        ax.set_title(f'{case}: ' + title.split(': ', 1)[1], fontsize=8.2, color=colour)
        ax.text(0.03, 0.80, f'bottle lifted {b[-1]:+.1f} cm\nmax tilt {tilt:.0f}°',
                transform=ax.transAxes, fontsize=8)
        ax.set_xlabel('time [s]')
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('height change [cm]')
    axes[0].legend(loc='center left', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'fig_attach_heights.png'), dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    fig_photos()
    fig_heights()
    print('figures written to', IMG)
