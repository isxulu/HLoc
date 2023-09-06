import argparse
import torch
from pathlib import Path
from typing import Dict, List, Union, Optional
import h5py
from types import SimpleNamespace
import cv2
import numpy as np
from tqdm import tqdm
import pprint
import collections.abc as collections
import PIL.Image
import glob

from . import extractors, logger
from .utils.base_model import dynamic_load
from .utils.parsers import parse_image_lists
from .utils.io import read_image, list_h5_names


'''
A set of standard configurations that can be directly selected from the command
line using their name. Each is a dictionary with the following entries:
    - output: the name of the feature file that will be generated.
    - model: the model configuration, as passed to a feature extractor.
    - preprocessing: how to preprocess the images read from disk.
'''
confs = {
    'superpoint_aachen': {
        'output': 'feats-superpoint-n4096-r1024',
        'model': {
            'name': 'superpoint',
            'nms_radius': 3,
            'max_keypoints': 4096,
        },
        'preprocessing': {
            'grayscale': True,
            'resize_max': 1024,
        },
    },
    # Resize images to 1600px even if they are originally smaller.
    # Improves the keypoint localization if the images are of good quality.
    'superpoint_max': {
        'output': 'feats-superpoint-n4096-rmax1600',
        'model': {
            'name': 'superpoint',
            'nms_radius': 3,
            'max_keypoints': 4096,
        },
        'preprocessing': {
            'grayscale': True,
            'resize_max': 1600,
            'resize_force': True,
        },
    },
    'superpoint_inloc': {
        'output': 'feats-superpoint-n4096-r1600',
        'model': {
            'name': 'superpoint',
            'nms_radius': 4,
            'max_keypoints': 4096,
        },
        'preprocessing': {
            'grayscale': True,
            'resize_max': 1600,
        },
    },
    'r2d2': {
        'output': 'feats-r2d2-n5000-r1024',
        'model': {
            'name': 'r2d2',
            'max_keypoints': 5000,
        },
        'preprocessing': {
            'grayscale': False,
            'resize_max': 1024,
        },
    },
    'd2net-ss': {
        'output': 'feats-d2net-ss',
        'model': {
            'name': 'd2net',
            'multiscale': False,
        },
        'preprocessing': {
            'grayscale': False,
            'resize_max': 1600,
        },
    },
    'sift': {
        'output': 'feats-sift',
        'model': {
            'name': 'dog'
        },
        'preprocessing': {
            'grayscale': True,
            'resize_max': 1600,
        },
    },
    'sosnet': {
        'output': 'feats-sosnet',
        'model': {
            'name': 'dog',
            'descriptor': 'sosnet'
        },
        'preprocessing': {
            'grayscale': True,
            'resize_max': 1600,
        },
    },
    'disk': {
        'output': 'feats-disk',
        'model': {
            'name': 'disk',
            'max_keypoints': 5000,
        },
        'preprocessing': {
            'grayscale': False,
            'resize_max': 1600,
        },
    },
    # Global descriptors
    'dir': {
        'output': 'global-feats-dir',
        'model': {'name': 'dir'},
        'preprocessing': {'resize_max': 1024},
    },
    'netvlad': {
        'output': 'global-feats-netvlad',
        'model': {'name': 'netvlad'},
        'preprocessing': {'resize_max': 1024},
    },
    'openibl': {
        'output': 'global-feats-openibl',
        'model': {'name': 'openibl'},
        'preprocessing': {'resize_max': 1024},
    },
    'cosplace': {
        'output': 'global-feats-cosplace',
        'model': {'name': 'cosplace'},
        'preprocessing': {'resize_max': 1024},
    }
}


def resize_image(image, size, interp):
    """
        修改图像尺寸

        参数:
            image: 待修改的图片
            size: 想要修改的尺寸
            interp: 插值方式

        返回值：
            修改尺寸后的图像
    """

    # 使用openCV中的插值方式
    if interp.startswith('cv2_'):
        interp = getattr(cv2, 'INTER_'+interp[len('cv2_'):].upper())
        h, w = image.shape[:2]
        if interp == cv2.INTER_AREA and (w < size[0] or h < size[1]):
            interp = cv2.INTER_LINEAR
        resized = cv2.resize(image, size, interpolation=interp)
    # 使用Python Image Lib中的插值方式
    elif interp.startswith('pil_'):
        interp = getattr(PIL.Image, interp[len('pil_'):].upper())
        resized = PIL.Image.fromarray(image.astype(np.uint8))
        resized = resized.resize(size, resample=interp)
        resized = np.asarray(resized, dtype=image.dtype)
    else:
        raise ValueError(
            f'Unknown interpolation {interp}.')
    
    return resized


class ImageDataset(torch.utils.data.Dataset):
    """
        定义图像数据集, 继承自torch中的数据集
    """
    default_conf = {
        'globs': ['*.jpg', '*.png', '*.jpeg', '*.JPG', '*.PNG'],
        'grayscale': False,
        'resize_max': None,
        'resize_force': False,
        'interpolation': 'cv2_area',  # pil_linear is more accurate but slower
    }

    def __init__(self, root, conf, paths=None):
        """
            数据集初始化，维护一个列表存储指定目录下的图像名称

            参数：
                root 读取图片的根目录
                conf 配置信息
                paths 路径
        """
        self.conf = conf = SimpleNamespace(**{**self.default_conf, **conf})
        self.root = root

        if paths is None:
            paths = []
            for g in conf.globs:
                paths += glob.glob(
                    (Path(root) / '**' / g).as_posix(), recursive=True)
            if len(paths) == 0:
                raise ValueError(f'Could not find any image in root: {root}.')
            paths = sorted(set(paths))
            self.names = [Path(p).relative_to(root).as_posix() for p in paths]
            logger.info(f'Found {len(self.names)} images in root {root}.')
        else:
            if isinstance(paths, (Path, str)):
                self.names = parse_image_lists(paths)
            elif isinstance(paths, collections.Iterable):
                self.names = [p.as_posix() if isinstance(p, Path) else p
                              for p in paths]
            else:
                raise ValueError(f'Unknown format for path argument {paths}.')

            for name in self.names:
                if not (root / name).exists():
                    raise ValueError(
                        f'Image {name} does not exists in root: {root}.')

    def __getitem__(self, idx):
        """
            依据索引idx取出数据集中对应的图片

            参数：
                idx 图片索引
        """

        # 拿到第idx张图片的名称
        name = self.names[idx]
        # 读取图片
        image = read_image(self.root / name, self.conf.grayscale)
        image = image.astype(np.float32)
        size = image.shape[:2][::-1]

        # 尺寸变化
        if self.conf.resize_max and (self.conf.resize_force
                                     or max(size) > self.conf.resize_max):
            scale = self.conf.resize_max / max(size)
            size_new = tuple(int(round(x*scale)) for x in size)
            image = resize_image(image, size_new, self.conf.interpolation)

        # 转化为torch中的数据格式，通道数，高度，宽度
        if self.conf.grayscale:
            image = image[None]
        else:
            image = image.transpose((2, 0, 1))  # HxWxC to CxHxW
        image = image / 255.

        data = {
            'image': image,
            'original_size': np.array(size),  # 返回的数据包含原始尺寸
        }
        return data

    def __len__(self):
        """
            返回数据集中元素的图片的数目
        """
        return len(self.names)


@torch.no_grad()
def main(conf: Dict,
         image_dir: Path,
         export_dir: Optional[Path] = None,
         as_half: bool = True,
         image_list: Optional[Union[Path, List[str]]] = None,
         feature_path: Optional[Path] = None,
         overwrite: bool = False) -> Path:
    logger.info('Extracting local features with configuration:'
                f'\n{pprint.pformat(conf)}')
    """
    特征提取的核心函数

    参数：
        conf 选用和中模型，模型中的参数
        image_dir 图像存储的路径
        export_dir 导出路径
        as_half 是否使用半精度
        image_list 图像名称列表
        feature_path 特征输出目录
        overwrite 是否覆盖
    """

    # 数据集
    dataset = ImageDataset(image_dir, conf['preprocessing'], image_list)

    if feature_path is None:
        feature_path = Path(export_dir, conf['output']+'.h5')
    feature_path.parent.mkdir(exist_ok=True, parents=True)

    skip_names = set(list_h5_names(feature_path)
                     if feature_path.exists() and not overwrite else ())
    dataset.names = [n for n in dataset.names if n not in skip_names]
    if len(dataset.names) == 0:
        logger.info('Skipping the extraction.')
        return feature_path

    # 判断设备，加载模型
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    Model = dynamic_load(extractors, conf['model']['name'])
    model = Model(conf['model']).eval().to(device)

    loader = torch.utils.data.DataLoader(
        dataset, num_workers=1, shuffle=False, pin_memory=True)
    for idx, data in enumerate(tqdm(loader)):
        name = dataset.names[idx]

        # 拿到特征，写入字典
        pred = model({'image': data['image'].to(device, non_blocking=True)})
        pred = {k: v[0].cpu().numpy() for k, v in pred.items()}

        # 添加图像大小属性
        pred['image_size'] = original_size = data['original_size'][0].numpy()
        # 如果提取到的特征中存在关键点属性
        if 'keypoints' in pred:
            size = np.array(data['image'].shape[-2:][::-1])
            scales = (original_size / size).astype(np.float32)  # 放缩比例
            pred['keypoints'] = (pred['keypoints'] + .5) * scales[None] - .5  # 将关键点的坐标算回缩放前
            if 'scales' in pred:
                pred['scales'] *= scales.mean()
            # add keypoint uncertainties scaled to the original resolution
            uncertainty = getattr(model, 'detection_noise', 1) * scales.mean()  # 关键点的不确定性与尺度关联

        # 半精度转换
        if as_half:
            for k in pred:
                dt = pred[k].dtype
                if (dt == np.float32) and (dt != np.float16):
                    pred[k] = pred[k].astype(np.float16)

        # 将特征写入hdf5文件
        with h5py.File(str(feature_path), 'a', libver='latest') as fd:
            try:
                if name in fd:
                    del fd[name]
                grp = fd.create_group(name)
                for k, v in pred.items():
                    grp.create_dataset(k, data=v)
                if 'keypoints' in pred:
                    grp['keypoints'].attrs['uncertainty'] = uncertainty
            except OSError as error:
                if 'No space left on device' in error.args[0]:
                    logger.error(
                        'Out of disk space: storing features on disk can take '
                        'significant space, did you enable the as_half flag?')
                    del grp, fd[name]
                raise error

        del pred

    logger.info('Finished exporting features.')
    return feature_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--image_dir', type=Path, required=True)
    parser.add_argument('--export_dir', type=Path, required=True)
    parser.add_argument('--conf', type=str, default='superpoint_aachen',
                        choices=list(confs.keys()))
    parser.add_argument('--as_half', action='store_true')
    parser.add_argument('--image_list', type=Path)
    parser.add_argument('--feature_path', type=Path)
    args = parser.parse_args()
    main(confs[args.conf], args.image_dir, args.export_dir, args.as_half)
