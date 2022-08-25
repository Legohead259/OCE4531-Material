#!/bin/bash

# Rename and move PDF
cp .latexoutput/main.pdf ../publish/lecture_notes_f22.pdf

# Move into the publish directory
cd ../publish/

# Chapter location declarations
doc_start=97
syllabus_length=7
p1_length=1
p2_length=1
p3_length=5
p4_length=5
icp_ug_length=5
icp_g_length=7
icsp_length=9


# Split PDF into chapters
# Syllabus
echo $doc_start # DEBUG
pdftk lecture_notes_f22.pdf cat $doc_start-$(( $doc_start + $syllabus_length )) output syllabus.pdf
doc_start=$(( $doc_start + $syllabus_length + 1)) # Increment tracker forward to next section

# Project 1
echo $doc_start # DEBUG
pdftk lecture_notes_f22.pdf cat $doc_start-$(( $doc_start + $p1_length )) output p1_rgb_led_cycler.pdf
doc_start=$(( $doc_start + $p1_length + 1)) # Increment tracker forward to next section

# Project 2
echo $doc_start # DEBUG
pdftk lecture_notes_f22.pdf cat $doc_start-$(( $doc_start + $p2_length )) output p2_digital_inputs.pdf
doc_start=$(( $doc_start + $p2_length + 1)) # Increment tracker forward to next section

# Project 3
echo $doc_start # DEBUG
pdftk lecture_notes_f22.pdf cat $doc_start-$(( $doc_start + $p3_length )) output p3_7seg_counter.pdf
doc_start=$(( $doc_start + $p3_length + 1)) # Increment tracker forward to next section

# Project 4
echo $doc_start # DEBUG
pdftk lecture_notes_f22.pdf cat $doc_start-$(( $doc_start + $p4_length )) output p4_accelerometer_display.pdf
doc_start=$(( $doc_start + $p4_length + 1)) # Increment tracker forward to next section

# ICP - Undergraduate
echo $doc_start # DEBUG
pdftk lecture_notes_f22.pdf cat $doc_start-$(( $doc_start + $icp_ug_length )) output icp_ugrad.pdf
doc_start=$(( $doc_start + $icp_ug_length + 1)) # Increment tracker forward to next section

# ICP - Graduate
echo $doc_start # DEBUG
pdftk lecture_notes_f22.pdf cat $doc_start-$(( $doc_start + $icp_g_length )) output icp_grad.pdf
doc_start=$(( $doc_start + $icp_g_length + 1)) # Increment tracker forward to next section

# ICSP Guide
echo $doc_start # DEBUG
pdftk lecture_notes_f22.pdf cat $doc_start-$(( $doc_start + $icsp_length )) output icsp_guide.pdf
# doc_start=$(( $doc_start + $p4_length + 1)) # Increment tracker forward to next section
