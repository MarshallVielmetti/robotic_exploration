from setuptools import find_packages, setup

package_name = "exploration_sim_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    # data_files=[]
    zip_safe=True,
    maintainer="Marshall Vielmetti",
    maintainer_email="mvielmet@umich.edu",
    description="Python utility package for exploration simulator",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_grapher = exploration_sim_utils.PathGrapher:main",
            "fit_splines = exploration_sim_utils.FitSplines:main",
            "connectivity_graph_plotter = exploration_sim_utils.ConnectivityGraphPlotter:main",
            "save_map = exploration_sim_utils.SaveMap:main",
            "atsp_solver = exploration_sim_utils.AtspSolver:main",
        ],
    },
)
