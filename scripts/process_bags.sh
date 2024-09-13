#!/bin/bash

DRY_RUN=false
PRINT_FULL=false
SEARCH_STR=
MODE="label"
BODIES_FILE=
OUTPUT="log"

for i in "$@"; do
	case $i in 
		--mode=*)
			MODE="${i#*=}"
			;;

		--out=*)
			OVERRIDE_OUTPUT_DIR="${i#*=}"
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

		--bodies=*)
			BODIES_FILE="${i#*=}"
			;;

		--verbose)
			OUTPUT="screen"
			;;

		*)
			# Default parameter is the bag directory
			PROJECT_DIR="${i}"
			;;
	esac
done

if [ -z "$PROJECT_DIR" ]; then
    echo "Directory not defined"
    echo "`base_name` <project_dir>: Process camera-feed bag files from project directory <project_dir> with tagslam."
	echo "--mode=: 'label' or 'localize'"
	echo "--out=: Desired output directory (full path?)"
	echo "--searh=: Search string for selecting bags to process from the specified directory"
	echo "--bodies=: File name for the yaml rosparam file containing the tag and body specifications"
	echo "--full: Print full file names"
	echo "--dry: Dry run: List out file names to prcoses but not actually process them"
	echo "--verbose: Print full tagslam processing output"
fi

if [[ "$MODE" == "label" ]]; then
	MODE_STR="labeled"
    BAG_DIR="$PROJECT_DIR/original"
    OUTPUT_DIR="$PROJECT_DIR/labeled"
elif [[ "$MODE" == "localize" ]] && ! [[ -z "$BODIES_FILE" ]]; then
	MODE_STR="localized"
    BAG_DIR="$PROJECT_DIR/labeled"
    OUTPUT_DIR="$PROJECT_DIR/localized"
else
	echo "Mode not defined or BODIES_FILE not defined"
	exit 0
fi

echo $BAG_DIR

if [ $OVERRIDE_OUTPUT_DIR ]; then
    OUTPUT_DIR=$OVERRIDE_OUTPUT_DIR
    echo "Overriding output directory"
fi

echo $OUTPUT_DIR

tput setaf 2
tput bold

for file in "$BAG_DIR"/*
do
    file_basename=`basename $file` # Save the base name of the file for naming / moving bag files
    file_basename=${file_basename#"labeled-"} # wtf is this command I have no idea what it does anymore

	if [[ $file =~ \.bag$ ]] && \
        [[ $file =~ $SEARCH_STR ]] && \
        [[ ! -f "$OUTPUT_DIR/$MODE_STR-$file_basename" ]] ; then
        # Print realpath if desired, just filename if not
        if [[ "$PRINT_FULL" == "true" ]]; then
            echo "Processing bag file: `realpath $file`" 
        else
            echo "Processing bag file: $file"
        fi

        # Process if not dry run
		if [[ "$DRY_RUN" == "false" ]]; then
            exec $MODE_CMD

			if [[ "$MODE" == "label" ]] && [[ "$file" != labeled* ]]; then
				roslaunch continuum_mocap sync_and_detect.launch bag:=`realpath $file` output:="$OUTPUT"
			elif [[ "$MODE" == "localize" ]] && [[ "$file" != localized* ]]; then
				roslaunch continuum_mocap tagslam.launch bag:=`realpath $file` output:="$OUTPUT" bodies_file:=$BODIES_FILE
			fi

		fi

		# If there's no explicitly defined output directory then exit the program
		# If not, move the output bag into the desired output directory
		if [ -z "$OUTPUT_DIR" ]; then
			continue
		fi

		# Print realpath if desired, just filename if not
        if [[ "$PRINT_FULL" == "true" ]]; then
            echo "Moving bag file to: `realpath $OUTPUT_DIR`/$MODE_STR-$file_basename" 
        else
            echo "Moving bag file to: $OUTPUT_DIR/$MODE_STR-$file_basename"
        fi
		# Move output if not dry run
		if [[ "$DRY_RUN" == "false" ]]; then
			# Hacky way to move the files to the desired directory
			# Assumes that we are using the default sync-and-detect file output name "<bag>_output.bag"
			if [[ "$MODE" == "label" ]]; then
				mv `realpath $file`_output.bag "`realpath $OUTPUT_DIR`/$MODE_STR-$file_basename"	
			elif [[ "$MODE" == "localize" ]]; then
				mv `realpath $file`_tagslam.bag "`realpath $OUTPUT_DIR`/$MODE_STR-$file_basename"
			fi
			
		fi
	else
		echo "Skipping $file - not eligible for processing"
	fi
done
