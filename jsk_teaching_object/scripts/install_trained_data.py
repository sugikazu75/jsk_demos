#!/usr/bin/env python

from __future__ import print_function

import argparse
import multiprocessing
import os.path as osp

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'jsk_teaching_object'

    download_data(
        pkg_name=PKG,
        path='trained_data/20220420-120316-dataset-20-daily-objects.tflite',
        url='https://drive.google.com/uc?id=1AvjThabb3ODTPC5SaAOKrRZ54-e3O8f8',
        md5='cfff1e0c6709c2fd4b67401232238246',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
