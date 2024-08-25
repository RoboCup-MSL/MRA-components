import glob

outputfile = "regression_suite_difference.html"
compare_dir_name = "compare_old"

# get svg files in compare directory
changed_files = [f for f in glob.glob(f"{compare_dir_name}/**/*.svg", recursive=True)]
# create svg_files list: remove directory name and suffix ".svg" from the changed files
svg_files = {x.replace(compare_dir_name+"/", "").replace(".svg","") for x in changed_files}
svg_files = list(svg_files)
svg_files.sort()

# create html page with difference
with open(outputfile, "w") as fp:
	fp.write("<!DOCTYPE html>\n")
	fp.write("<html>\n")
	fp.write("<body>\n")
	fp.write(f"Teamplanner regression suite differences: {len(svg_files)}<BR>\n")
	fp.write("<table border=\"5\">\n")
	diff_nr = 0
	for svg_file in svg_files:
		diff_nr = diff_nr + 1
		fp.write(f"  <tr><td colspan=\"2\"><h1>difference #{diff_nr}: {svg_file}</h1></td></tr>\n")
		fp.write("  <tr>\n")
		fp.write(f"   	<td><h2>Previous:</h2><img src=\"./compare_old/{svg_file}.svg\" alt=\"{svg_file}: before\"></td>\n")
		fp.write(f"   	<td><h2>Updated:</h2><img src=\"./compare_new/{svg_file}.svg\" alt=\"{svg_file}: after\"></td>\n")
	fp.write("</table>\n")
	fp.write("</body>\n")
	fp.write("</html>\n")
print(f"Create HTML to compare difference in regression suite: {outputfile}")
