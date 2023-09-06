from pathlib import Path
import argparse
from shutil import copyfile
from os import system



def arg_parse():
    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument('--path1', type=str, required=True, help='Absolute path of data1.')
    arg_parser.add_argument('--path2', type=str, required=True, help='Absolute path of data2.')
    arg_parser.add_argument("--out_path_name", type=str, default='data_', help='Default out path name.')
    arg_parser.add_argument('--thresh_hold', type=int, default=10000, help='The maximum time difference that can be tolerated when synchronizing the left and right cameras [ns]')

    args = arg_parser.parse_args()

    return args


def find_min(img0, imgs):
    min_diff, res = 99999999999999999999, None
    for img in imgs:
        if abs(int(img0[:-4])-int(img[:-4])) < min_diff:
            min_diff = abs(int(img0[:-4])-int(img[:-4]))
            res = img

    return res, min_diff


def main():
    args = arg_parse()

    path1, path2 = Path(args.path1), Path(args.path2)
    system(f'rm -rf {str((path1.parent/args.out_path_name))}')
    system(f'rm -rf {str((path2.parent/args.out_path_name))}')
    (path1.parent/args.out_path_name).mkdir()
    (path2.parent/args.out_path_name).mkdir()
    npath1, npath2 = path1.parent / args.out_path_name, path2.parent / args.out_path_name
    imgs1 = [(str(p).split('/'))[-1] for p in path1.iterdir()]
    imgs2 = [(str(p).split('/'))[-1] for p in path2.iterdir()]
    imgs1.sort(); imgs2.sort()

    imgs1_new, imgs2_new = [], []    
    thresh_hold = args.thresh_hold  # ns
    for img1 in imgs1:
        img2, diff = find_min(img1, imgs2)

        if diff < thresh_hold:
            imgs1_new.append(img1)
            imgs2_new.append(img2)
    print(f'Total {len(imgs1_new)} frames are reserved.')

    print(imgs1_new[-1], imgs2_new[-1])

    for i in range(len(imgs1_new)):
        copyfile(str(path1/imgs1_new[i]), str(npath1/imgs1_new[i]))
        copyfile(str(path2/imgs2_new[i]), str(npath2/imgs1_new[i]))



if __name__ == '__main__':
    main()

