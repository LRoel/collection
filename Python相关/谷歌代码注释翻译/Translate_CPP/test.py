import re
import os
import shutil
import json
import string
from translate import translate_it

g = os.walk("./cartographer")
if not os.path.isdir("./cartographer_bak"):
	os.mkdir("./cartographer_bak")

for path, d, filelist in g:
	for filename in filelist:
		print(path)