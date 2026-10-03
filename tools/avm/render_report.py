"""Render standalone algorithm figures from generated fixtures (requires matplotlib)."""
import json
from pathlib import Path
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]


def main():
    target=ROOT/'doc/avm/figures';target.mkdir(parents=True,exist_ok=True)
    for case in ('synthetic720','synthetic720-wide','public'):
        source=ROOT/'sim/data'/case
        fig,axes=plt.subplots(2,4,figsize=(16,7.8),constrained_layout=True)
        for ax,name in zip(axes[0],('front','back','left','right')):
            ax.imshow(cv2.imread(str(source/f'{name}_raw.png'))[...,::-1]);ax.set_title(name);ax.axis('off')
        axes[1,0].imshow(cv2.imread(str(source/'golden.png'))[...,::-1]);axes[1,0].set_title('Q8 mesh / RGB565 reference');axes[1,0].axis('off')
        arrays=np.load(source/'algorithm_arrays.npz')
        err=np.ma.array(arrays['error'],mask=~arrays['valid'].any(0))
        im=axes[1,1].imshow(err,cmap='turbo',vmin=0,vmax=.25);axes[1,1].set_title('Source-coordinate error');axes[1,1].axis('off')
        fig.colorbar(im,ax=axes[1,1],label='pixels',shrink=.8)
        metrics=json.loads((source/'metrics.json').read_text())
        hist=metrics['grid']['step_histogram'];axes[1,2].bar(list(hist),list(hist.values()),color='#2d688a')
        axes[1,2].set_title('Adaptive grid spacing');axes[1,2].set_xlabel('step (output pixels)');axes[1,2].set_ylabel('active camera patches')
        axes[1,3].axis('off')
        text=(f"Output: 1280 x 720\n\nMax map error: {metrics['grid']['max_coordinate_error_px']:.6f} px\n"
              f"Node bytes: {metrics['grid']['node_bytes']:,}\n"
              f"PSNR: {metrics['quality_vs_float']['psnr_db']:.3f} dB\n"
              f"SSIM: {metrics['quality_vs_float']['ssim']:.6f}\n\n"
              "MATLAB independent full-frame\nfloat reference: exact RGB match\n\n"
              "PSNR/SSIM evaluated on visible\npixels against double reference.\n"
              "RGB565 quantization is included.")
        axes[1,3].text(0,.95,text,va='top',fontsize=10)
        fig.suptitle(f'AVM algorithm validation — {case}',fontsize=17)
        fig.savefig(target/f'{case}_algorithm.png',dpi=150);plt.close(fig)
        cv2.imwrite(str(target/f'{case}_bev.png'),cv2.imread(str(source/'golden.png')))
    fig,axes=plt.subplots(1,2,figsize=(12,4.2),constrained_layout=True)
    palette=np.array([[.9,.15,.15],[.15,.3,.95],[.95,.75,.12],[.2,.8,.3]])
    for ax,case in zip(axes,('synthetic720-wide','synthetic720')):
        route=np.load(ROOT/'sim/data'/case/'algorithm_arrays.npz')['routes']
        a=route&3;b=(route>>2)&3;va=(route&16)!=0;vb=(route&32)!=0;alpha=((route>>6)&511)/256
        color=palette[a]*alpha[...,None]*va[...,None]+palette[b]*(1-alpha[...,None])*vb[...,None]
        ax.imshow(color);ax.axis('off')
        ax.set_title(f'{case}: {int(vb.sum()):,} two-camera pixels')
    fig.suptitle('Same cameras and mapping; different seam policies')
    fig.savefig(target/'seam_policy_comparison.png',dpi=180);plt.close(fig)
    data=json.loads((ROOT/'doc/avm/exposure-feedback.json').read_text())
    history=np.array(data['history']);fig,ax=plt.subplots(figsize=(8,4.2),constrained_layout=True)
    for c,name in enumerate(('front','back','left','right')):
        ax.plot(np.arange(1,len(history)+1),history[:,c,0],label=f'{name} R')
    ax.set(xlabel='Host updates (5 Hz)',ylabel='Gain',title='Gain feedback from actual RTL overlap statistics')
    ax.grid(alpha=.25);ax.legend(ncol=4)
    fig.savefig(target/'exposure_convergence.png',dpi=180);plt.close(fig)
    print(target)


if __name__=='__main__':main()
