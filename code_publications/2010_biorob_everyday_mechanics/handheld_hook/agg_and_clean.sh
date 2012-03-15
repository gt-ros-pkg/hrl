cd aggregated_pkls

rm -vf *open_open*
rm -vf *open_close*
rm -vf *close_open*
rm -vf *close_close*

cd ..
python mechanism_analyse_advait.py -d aggregated_pkls --rearrange
cd aggregated_pkls/HSI_kitchen_cabinet_right

mv *aaron* ../HSI_kitchen_cabinet_right_aaron/
mv *hai* ../HSI_kitchen_cabinet_right_hai/
mv *tiffany* ../HSI_kitchen_cabinet_right_tiffany/
mv *charlie* ../HSI_kitchen_cabinet_right_charlie/

cd ../../
python mechanism_analyse_advait.py -d aggregated_pkls --clean




