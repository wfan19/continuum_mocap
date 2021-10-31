#!/bin/bash

DRY_RUN=false
PRINT_FULL=false
SEARCH_STR=

for i in "$@"; do
	case $i in 
		--dry)
			DRY_RUN=true
			shift
			;;

		--search=*)
			# Pass search string
			SEARCH_STR="${i#*=}"
            shift
            ;;

		--full)
			# Toggle whether to print the full file path
			PRINT_FULL=true
			shift
			;;

		*)
			# Default parameter is the bag directory
			BAG_DIR="${i}"
			;;
	esac
done

if [ -z "$BAG_DIR" ]; then
    echo "Directory not defined"
    exit 0
fi

for file in "$BAG_DIR"/*
do
	if [[ $file != *"bag_output"*  ]] && [[ $file =~ \.bag$ ]] && [[ $file =~ $SEARCH_STR ]]; then
        # Print realpath if desired, just filename if not
        if [[ "$PRINT_FULL" == "true" ]]; then
            echo "Processing bag file: `realpath $file`" 
        else
            echo "Processing bag file: $file"
        fi

        # Process if not dry run
		if [[ "$DRY_RUN" == "false" ]]; then
            roslaunch continuum_mocap sync_and_detect.launch bag:=`realpath $file`
		fi
	fi
done
