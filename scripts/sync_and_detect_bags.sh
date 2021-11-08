#!/bin/bash

DRY_RUN=false
PRINT_FULL=false
SEARCH_STR=

for i in "$@"; do
	case $i in 
		--out=*)
			OUTPUT_DIR="${i#*=}"
			;;

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
            roslaunch continuum_mocap sync_and_detect.launch bag:=`realpath $file` output:="log"

			# Hacky way to move the files to the desired directory
			# Assumes that we are using the default sync-and-detect file output name "<bag>_output.bag"
		fi

		# If there's no explicitly defined output directory then exit the program
		# If not, move the output bag into the desired output directory
		if [ -z "$OUTPUT_DIR" ]; then
			exit 0
		fi

		# Print realpath if desired, just filename if not
        if [[ "$PRINT_FULL" == "true" ]]; then
            echo "Moving bag file to: `realpath $OUTPUT_DIR`" 
        else
            echo "Moving bag file to: $OUTPUT_DIR"
        fi
		# Move output if not dry run
		if [[ "$DRY_RUN" == "false" ]]; then
			# Hacky way to move the files to the desired directory
			# Assumes that we are using the default sync-and-detect file output name "<bag>_output.bag"
			mv `realpath $file`_output.bag `realpath $OUTPUT_DIR`
		fi
	fi
done