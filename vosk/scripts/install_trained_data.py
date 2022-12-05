#!/usr/bin/env python

import argparse
import multiprocessing

import jsk_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    def download_data(**kwargs):
        kwargs['pkg_name'] = 'vosk'
        kwargs['quiet'] = quiet
        p = multiprocessing.Process(
            target=jsk_data.download_data,
            kwargs=kwargs)
        p.start()

    download_data(
        path='models/vosk-model-small-ja-0.22.zip',
        url='https://alphacephei.com/vosk/models/vosk-model-small-ja-0.22.zip',
        md5='0e3163dd62dfb0d823353718ac3cbf79',
        extract=True,
    )


if __name__ == '__main__':
    main()
