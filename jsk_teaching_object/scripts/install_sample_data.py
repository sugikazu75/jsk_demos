#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_teaching_object'

    download_data(
        pkg_name=PKG,
        path='sample/data/daily-objects-labels.txt',
        url='https://drive.google.com/uc?id=1QkXxdSkIjivgcBWADa6tNZtSQVl-6Pro',
        md5='16066e1e9aab0035ee036d65d268e5f3',
        extract=False,
    )


if __name__ == '__main__':
    main()
