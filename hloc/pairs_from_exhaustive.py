import argparse
import collections.abc as collections
from pathlib import Path
from typing import Optional, Union, List

from . import logger
from .utils.parsers import parse_image_lists
from .utils.io import list_h5_names


def main(
        output: Path,
        image_list: Optional[Union[Path, List[str]]] = None,
        features: Optional[Path] = None,
        ref_list: Optional[Union[Path, List[str]]] = None,
        ref_features: Optional[Path] = None):
    """
    构建后续用于特征匹配的图像对，并将匹配的结果存储在文件中
    exhaustive表明任意两对之间都需要进行特征匹配(在实际应用中,可能很难承受这么大的代价,在demo中没问题)

    参数：
        output 输出路径
        image_list 图像名称/路径列表
        features 特征文件路径
        ref_list 参考图像列表
        ref_features 参考特征

        在图像和特征中仅需要提供一种参数即可,如在demo中使用的是image
    """

    # 提供了图像列表
    if image_list is not None:
        # 取出图像的路径或名称放在names_q列表中
        if isinstance(image_list, (str, Path)):
            names_q = parse_image_lists(image_list)
        elif isinstance(image_list, collections.Iterable):
            names_q = list(image_list)
        else:
            raise ValueError(f'Unknown type for image list: {image_list}')
    # 提供了特征
    elif features is not None:
        # names_q拿到的是特征列表
        names_q = list_h5_names(features)
    else:
        raise ValueError('Provide either a list of images or a feature file.')

    self_matching = False  # 是否进行自匹配

    # 判断是否提供了参考，若提供了，将名称存在names_ref中
    if ref_list is not None:
        if isinstance(ref_list, (str, Path)):
            names_ref = parse_image_lists(ref_list)
        elif isinstance(image_list, collections.Iterable):
            names_ref = list(ref_list)
        else:
            raise ValueError(
                f'Unknown type for reference image list: {ref_list}')
    elif ref_features is not None:
        names_ref = list_h5_names(ref_features)
    # 若没有提供，则说明需要进行自匹配（传入的图像之间进行匹配）
    else:
        self_matching = True
        # 参考和待匹配是同样的图片
        names_ref = names_q

    # 获得图像对，自匹配避免重复
    pairs = []
    for i, n1 in enumerate(names_q):
        for j, n2 in enumerate(names_ref):
            if self_matching and j <= i:
                continue
            pairs.append((n1, n2))

    # 写入文件
    logger.info(f'Found {len(pairs)} pairs.')
    with open(output, 'w') as f:
        f.write('\n'.join(' '.join([i, j]) for i, j in pairs))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--image_list', type=Path)
    parser.add_argument('--features', type=Path)
    parser.add_argument('--ref_list', type=Path)
    parser.add_argument('--ref_features', type=Path)
    args = parser.parse_args()
    main(**args.__dict__)
