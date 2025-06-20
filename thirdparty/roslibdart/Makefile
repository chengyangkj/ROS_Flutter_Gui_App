.PHONY: docs build

gitrcommit:
	git config --global credential.helper store
	# git add -u
	
	-git add example/*/* example/*/*/*
	-git add Makefile README lib lib/* pubspec.yaml
	-git add rospackage rospackage/* rospackage/*/*
	-git commit -a -m "`date`"
	git pull
	git push origin HEAD
gitrupdate:
	git config --global credential.helper store
	git pull
dartpublish:
	dart pub publish 
