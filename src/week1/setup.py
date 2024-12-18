from setuptools import find_packages, setup

package_name = "week1"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="htanmo",
    maintainer_email="145841395+htanmo@users.noreply.github.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "integer_generator = week1.integer_generator:main",
            "odd_even_classifier = week1.odd_even_classifier:main",
        ],
    },
)
