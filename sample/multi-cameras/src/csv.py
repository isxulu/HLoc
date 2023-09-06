import argparse
from pathlib import Path


def parse_args():
    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument("--path", default='.', type=str, help='Path of data dir')
    arg_parser.add_argument("--name", default='data', type=str, help='Name of csv file')

    args = arg_parser.parse_args()

    return args


def main():
    args = parse_args()

    data_path = Path(args.path)
    figs = [str(fig).split('/')[-1] for fig in data_path.iterdir()]

    print(str(Path(args.path).parent)+"/data.csv")
    with open(str(Path(args.path).parent)+"/data.csv", "w") as f:
        f.write('# timestamp [ns],filename\n')
        for fig in figs:
            f.write(fig[:-4]+","+fig+"\n")


if __name__ == '__main__':
    main()

