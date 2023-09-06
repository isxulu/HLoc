import argparse
from os import system
from pathlib import Path
from typing import Union


def main(images_path: Union[str, Path], out_path: Union[str, Path], step: int) -> None:
    if isinstance(images_path, str):
        images_path: Path = Path(images_path)
    if isinstance(out_path, str):
        out_path: Path = Path(out_path)

    if out_path.exists():
        system(f'rm -rf {str(out_path.absolute())}')
    out_path.mkdir()

    save_images = []

    # convert generator to list for sequential information
    for i, image in enumerate(sorted(list(images_path.iterdir())), start=1):
        if i%step == 0:
            save_images.append(image)

    print(f"{len(save_images)} frames have been extracted.", end='\n\n')

    for i, image in enumerate(save_images, start=1):
        dest = out_path / f'{i:05d}.png'
        dest.write_bytes(image.read_bytes())

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('--step', default=10, type=int)
    parser.add_argument('--images_path', default="", type=str)
    parser.add_argument('--out_path', default='', type=str)

    args = parser.parse_args()

    main(**args.__dict__)