#!/bin/bash

./../process_friis_plots.py --yaml friis_plot_shoulder_table_both_OrangeMedBot.yaml
./../process_friis_plots.py --yaml friis_plot_shoulder_table_both_OnMetalKeys.yaml
./../process_friis_plots.py --yaml friis_plot_shoulder_table_both_SpectrMedBot.yaml
./../process_friis_plots.py --yaml friis_plot_shoulder_table_both_TravisTVremo.yaml
./../process_friis_plots.py --yaml friis_plot_shoulder_table_both_RedMug.yaml

./../process_radpat_plots.py --yaml rad_plot_shoulder_table_both_OrangeMedBot.yaml
./../process_radpat_plots.py --yaml rad_plot_shoulder_table_both_OnMetalKeys.yaml
./../process_radpat_plots.py --yaml rad_plot_shoulder_table_both_SpectrMedBot.yaml
./../process_radpat_plots.py --yaml rad_plot_shoulder_table_both_TravisTVremo.yaml
./../process_radpat_plots.py --yaml rad_plot_shoulder_table_both_RedMug.yaml