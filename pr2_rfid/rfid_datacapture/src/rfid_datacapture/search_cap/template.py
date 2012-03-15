x = '''
\\begin{landscape}
\\begin{table}[p]
  \\newcolumntype{x}[1]{>{\\centering\\hspace{0pt}}p{#1}}
  \\newcolumntype{y}[1]{>{\\raggedleft\\hspace{0pt}}p{#1}}
  \\centering
  \\begin{tabular}{ | y{2cm} | c | c | c | c | r }
    \\toprule
    \\textbf{\\Large Object:} & Orange Meds & TV Remote & Red Bottle & Keys & \\multirow{4}{1.0cm}{\\LARGE ...} \\\\
    \\midrule
    \\midrule
    \\textbf{Reads}             & %s & %s & %s & %s  & \\\\
    \\textbf{$\\Delta xy$}      & %s & %s & %s & %s  & \\\\
    \\textbf{$\\Delta \\Theta$} & %s & %s & %s & %s  & \\\\
    \\bottomrule
  \\end{tabular}

  \\vskip 30pt

  \\begin{tabular}{ c | c | c | c | c | c | }
    \\toprule
    \\multirow{4}{0.5cm}{\\LARGE ...} & Med Bottle & Med Box & Teddy Bear & Cordless Phone & Hair Brush \\\\
    \\midrule
    \\midrule
    & %s & %s & %s & %s & %s \\\\
    & %s & %s & %s & %s & %s \\\\
    & %s & %s & %s & %s & %s \\\\
    \\bottomrule
  \\end{tabular}

  \\vskip 30pt

  \\begin{tabular}{r r@{ = }l r@{ = }l}
    \\toprule
    \\multicolumn{5}{c}{\\textbf{\\Large OVERALL FOR LOCATION \\#%d}}\\\\
    \\midrule
    \\midrule
    \\textbf{Reads:} & $\\mu$ & %3.1f & $\\sigma$ & %3.1f \\\\
    \\textbf{$\\Delta xy$} & $\\mu$ & %3.1f m & $\\sigma$ & %3.1f m \\\\
    \\textbf{$|\\Delta \\Theta|$} & $\\mu$ & %3.1f$^o$ & $\\sigma$ & %3.1f$^o$ \\\\
    \\bottomrule
  \\end{tabular}

  \\caption{ RFID Search Performance for Tagged Objects at Location \\#%d }
  \\label{tab:search-results-%d}
\end{table}
\end{landscape}
'''
