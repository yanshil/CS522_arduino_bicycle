## The powershell script to create tretrahedral mesh from stl files

mkdir ".\tri_msh\"

## For each stl file in this folder
Get-ChildItem ${pwd}\bin_stl\ -Filter "*.stl" |
	ForEach-Object {
		docker run --rm -v ${pwd}:/data yixinhu/tetwild ./bin_stl/"$_" ./tri_msh/"$_".msh
	}

Remove-Item .\tri_msh\*.obj
Remove-Item .\tri_msh\*.csv
