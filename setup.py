from setuptools import setup, find_packages, Command
import os

class CleanCommand(Command):
    """Custom clean command to tidy up the project root."""
    user_options = []
    def initialize_options(self):
        pass
    def finalize_options(self):
        pass
    def run(self):
        os.system('rm -vrf ./build ./dist ./*.pyc ./*.tgz ./*.egg-info')

setup(
	name="dwa-jj",
	version="1.0",
	packages=find_packages(),
	install_requires=[
		line.strip() for line in open("requirments.txt")
	],
    	cmdclass={
        'clean': CleanCommand,
    	}

	
)
