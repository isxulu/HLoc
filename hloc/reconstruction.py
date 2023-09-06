import argparse
import shutil
from typing import Optional, List, Dict, Any
import multiprocessing
from pathlib import Path
import pycolmap

from . import logger
from .utils.database import COLMAPDatabase
from .triangulation import (
    import_features, import_matches, estimation_and_geometric_verification,
    OutputCapture, parse_option_args)


def create_empty_db(database_path: Path):
    """
        在Path路径下, 创建空的数据库
    """
    if database_path.exists():
        logger.warning('The database already exists, deleting it.')
        database_path.unlink()
    logger.info('Creating an empty database...')
    db = COLMAPDatabase.connect(database_path)
    db.create_tables()
    db.commit()
    db.close()


def import_images(image_dir: Path,
                  database_path: Path,
                  camera_mode: pycolmap.CameraMode,
                  image_list: Optional[List[str]] = None,
                  options: Optional[Dict[str, Any]] = None):
    """
        将图像引入数据库

        参数：
            image_dir 图像目录
            database_path 数据库路径
            camera_mode 相机模式
            image_list 图像列表
            options 图像操作
    """
    logger.info('Importing images into the database...')

    if options is None:
        options = {}

    images = list(image_dir.iterdir())

    if len(images) == 0:
        raise IOError(f'No images found in {image_dir}.')
    with pycolmap.ostream():
        # 调用pycolmap方法引入图片
        pycolmap.import_images(database_path, image_dir, camera_mode,
                               image_list=image_list or [],
                               options=options)


def get_image_ids(database_path: Path) -> Dict[str, int]:
    """
        依据数据库，构建 图像名称：索引 字典
    """
    db = COLMAPDatabase.connect(database_path)
    images = {}
    for name, image_id in db.execute("SELECT name, image_id FROM images;"):
        images[name] = image_id
    db.close()
    return images


def run_reconstruction(sfm_dir: Path,
                       database_path: Path,
                       image_dir: Path,
                       verbose: bool = False,
                       options: Optional[Dict[str, Any]] = None,
                       ) -> pycolmap.Reconstruction:
    """
        依据数据库，构建模型

        参数：
            sfm_dir 保存模型结果的路径
            database_path 数据库路径
            image_dir 图像路径
            verbose 是否输出详细信息
            options 重建模型的额外参数选项
    """

    models_path = sfm_dir / 'models'
    models_path.mkdir(exist_ok=True, parents=True)
    logger.info('Running 3D reconstruction...')
    if options is None:
        options = {}
    
    # 多线程
    options = {'num_threads': min(multiprocessing.cpu_count(), 16), **options}

    # 建图
    with OutputCapture(verbose):
        with pycolmap.ostream():
            reconstructions = pycolmap.incremental_mapping(
                database_path, image_dir, models_path, options=options)

    if len(reconstructions) == 0:
        logger.error('Could not reconstruct any model!')
        return None
    logger.info(f'Reconstructed {len(reconstructions)} model(s).')

    # 寻找最大模型的索引
    largest_index = None
    largest_num_images = 0
    for index, rec in reconstructions.items():
        num_images = rec.num_reg_images()
        if num_images > largest_num_images:
            largest_index = index
            largest_num_images = num_images
    assert largest_index is not None
    logger.info(f'Largest model is #{largest_index} '
                f'with {largest_num_images} images.')

    # 将结果写入文件
    for filename in ['images.bin', 'cameras.bin', 'points3D.bin']:
        if (sfm_dir / filename).exists():
            (sfm_dir / filename).unlink()
        shutil.move(
            str(models_path / str(largest_index) / filename), str(sfm_dir))
        
    # 返回最大的模型
    return reconstructions[largest_index]


def main(sfm_dir: Path,
         image_dir: Path,
         pairs: Path,
         features: Path,
         matches: Path,
         camera_mode: pycolmap.CameraMode = pycolmap.CameraMode.AUTO,
         verbose: bool = False,
         skip_geometric_verification: bool = False,
         min_match_score: Optional[float] = None,
         image_list: Optional[List[str]] = None,
         image_options: Optional[Dict[str, Any]] = None,
         mapper_options: Optional[Dict[str, Any]] = None,
         ) -> pycolmap.Reconstruction:
    """
        模型重建的主函数

        参数：
            sfm_dir 模型输出路径
            image_dir 图像路径
            pairs 图相匹配对
            fratures 特征目录
            matches 特征匹配的结果目录
            camera_mode 相机模式
            verbose 是否输出详细信息
            skil_geometric_verification 是否跳过几何验证
            min_match_score 最小匹配分数
            image_list 图像列表
            image_options 图像操作
            mapper_options
    """

    # 判断相应的目录是否存在
    assert features.exists(), features
    assert pairs.exists(), pairs
    assert matches.exists(), matches

    # 创建模型目录，构建数据库路径
    sfm_dir.mkdir(parents=True, exist_ok=True)
    database = sfm_dir / 'database.db'

    # 创建数据库
    create_empty_db(database)
    # 将图片加入数据库
    import_images(image_dir, database, camera_mode, image_list, image_options)
    # 建立『图像：索引字典』
    image_ids = get_image_ids(database)
    # 将特征导入数据库
    import_features(image_ids, database, features)
    # 将特征匹配的结果导入数据库
    import_matches(image_ids, database, pairs, matches,
                   min_match_score, skip_geometric_verification)
    
    # 若没有跳过几何验证，在数据库中进行几何验证
    if not skip_geometric_verification:
        estimation_and_geometric_verification(database, pairs, verbose)

    # 依据参数，构建模型
    reconstruction = run_reconstruction(sfm_dir, database, image_dir, verbose, mapper_options)
    if reconstruction is not None:
        logger.info(f'Reconstruction statistics:\n{reconstruction.summary()}'
                    + f'\n\tnum_input_images = {len(image_ids)}')
        
    return reconstruction


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sfm_dir', type=Path, required=True)
    parser.add_argument('--image_dir', type=Path, required=True)

    parser.add_argument('--pairs', type=Path, required=True)
    parser.add_argument('--features', type=Path, required=True)
    parser.add_argument('--matches', type=Path, required=True)

    parser.add_argument('--camera_mode', type=str, default="AUTO",
                        choices=list(pycolmap.CameraMode.__members__.keys()))
    parser.add_argument('--skip_geometric_verification', action='store_true')
    parser.add_argument('--min_match_score', type=float)
    parser.add_argument('--verbose', action='store_true')

    parser.add_argument('--image_options', nargs='+', default=[],
                        help='List of key=value from {}'.format(
                            pycolmap.ImageReaderOptions().todict()))
    parser.add_argument('--mapper_options', nargs='+', default=[],
                        help='List of key=value from {}'.format(
                            pycolmap.IncrementalMapperOptions().todict()))
    args = parser.parse_args().__dict__

    image_options = parse_option_args(
        args.pop("image_options"), pycolmap.ImageReaderOptions())
    mapper_options = parse_option_args(
        args.pop("mapper_options"), pycolmap.IncrementalMapperOptions())

    main(**args, image_options=image_options, mapper_options=mapper_options)
