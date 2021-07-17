from setuptools import setup, find_packages


setup(
    name="witmotion",
    version="0.0.1.dev",
    description="API for Witmotion IMUs, including HWT905",
    long_description="",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
    ],
    keywords="witmotion hwt905 imu",
    url="https://github.com/storborg/witmotion",
    author="Scott Torborg",
    author_email="storborg@gmail.com",
    license="MIT",
    packages=find_packages(),
    install_requires=[
        "pyserial",
    ],
    include_package_data=True,
    zip_safe=False,
    entry_points="""\
      [console_scripts]
      witmotion-debug = witmotion.cmd.debug:main
      """,
)
