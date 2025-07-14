#!/bin/bash

# Syntax printing
function print_syntax() {
    echo
    echo "Usage: $0 <MAX_NBR_GROUPS> <MAX_NBR_STATES> <MAX_NBR_CONNECTIONS> <TXT_FILE>"
    echo
    exit 1
}

function write_state() {
  INDEX=$1
  NB_TRANS=$2
  GROUP=$3
  echo "S${INDEX}_$GROUP     \"--s${INDEX}_$GROUP \"  c   (0,1,2,3,4,5) | as.numeric(NumStates_$GROUP)>$INDEX " >> ${TXT_FILE}
  echo "VEL${INDEX}_$GROUP   \"--vel${INDEX}_$GROUP \"  r (0.1,1.5) | as.numeric(S${INDEX}_$GROUP)==8 " >> ${TXT_FILE}
  echo "CLE${INDEX}_$GROUP   \"--cle${INDEX}_$GROUP \"  i (0,10)" >> ${TXT_FILE}
  echo "CLR${INDEX}_$GROUP   \"--clr${INDEX}_$GROUP \"  i (0,10)" >> ${TXT_FILE}
  echo "NumConnections${INDEX}_$GROUP \"--n${INDEX}_$GROUP \" i (1,$NB_TRANS) | as.numeric(NumStates_$GROUP)>$INDEX" >> ${TXT_FILE}
}

function write_connection() {
  STATE=$1
  CONNECTION=$2
  GROUP=$3
  echo "N${STATE}x${CONNECTION}_$GROUP  \"--n${STATE}x${CONNECTION}_$GROUP \" i   (0,3) | as.numeric(NumConnections${STATE}_$GROUP)>$CONNECTION " >> ${TXT_FILE}
  echo "C${STATE}x${CONNECTION}_$GROUP  \"--c${STATE}x${CONNECTION}_$GROUP \" c   (0,1,2,3,4,5) | as.numeric(NumConnections${STATE}_$GROUP)>$CONNECTION " >> ${TXT_FILE}
  echo "P${STATE}x${CONNECTION}_$GROUP  \"--p${STATE}x${CONNECTION}_$GROUP \" r   (0,1) | as.numeric(C${STATE}x${CONNECTION}_$GROUP) %in% c(0,1,2,5) " >> ${TXT_FILE}
  echo "L${STATE}x${CONNECTION}_$GROUP  \"--l${STATE}x${CONNECTION}_$GROUP \" i   (0,10) | as.numeric(C${STATE}x${CONNECTION}_$GROUP)==5 " >> ${TXT_FILE}
}

if [ $# -lt 4 ]; then
    print_syntax
fi

MAX_NBR_GROUPS=$(( $1 - 1 ))
MAX_NBR_STATES=$(( $2 - 1 ))
MAX_NBR_CONNECTIONS=$(( $3 - 1 ))
TXT_FILE=$4

truncate -s 0 $TXT_FILE

for GROUP in $(seq 0 $MAX_NBR_GROUPS)
do
  echo "NumStates_$GROUP   \"--nstates_$GROUP \"   i (1,$2)" >> $TXT_FILE
  for STATE in $(seq 0 $MAX_NBR_STATES)
  do
    write_state $STATE $3 $GROUP
    for CONNECTION in $(seq 0 $MAX_NBR_CONNECTIONS)
    do
      write_connection $STATE $CONNECTION $GROUP
    done
  done
done
