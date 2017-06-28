%%
%% This is the file 'ion.cls' intended for formatting submissions to
%% the Proceedings of the IoN GNSS meetings,
%%
%% Your class definition should be 
%%
%%     \documentclass{ion}, or
%%     \documentclass[ugly]{ion} if you want to produce the official
%%          layout.
%%     \documentclass[a4]{ion} outputs to A4 size paper and localises
%%          to British English.
%%
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{ion}
          [2006/05/23 v0.1 IoN GNSS Proceedings format]
% Options:
\newif\if@ugly\@uglyfalse
\newif\if@afour\@afourfalse
\DeclareOption{ugly}{\@uglytrue}
\DeclareOption{a4}{\@afourtrue}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax
%
\if@afour
	\LoadClass[a4paper,10pt]{article}
\else
	\LoadClass[letterpaper,10pt]{article}
\fi
\RequirePackage{times}
\RequirePackage{mathptm}
\RequirePackage{apalike}
\RequirePackage[hypertex]{hyperref}
\RequirePackage[\if@afour english\else american\fi]{babel}

% Redefine \maketitle:
\renewcommand\maketitle{\onecolumn\par
%%  \ifnum \col@number=\@ne
%%     \@maketitle
%%  \else
     \@maketitle%
%%  \fi
 \global\let\maketitle\relax
 \global\let\@maketitle\relax
 \global\let\@author\@empty
 \global\let\@date\@empty
 \global\let\@title\@empty
 \global\let\title\relax
 \global\let\author\relax
 \global\let\date\relax
 \global\let\and\relax
}
%
\def\@maketitle{%
 \vskip 0.25in%
   {%
   \par\normalfont\Huge\bfseries%
     \centering%
   \@title\par}%
   \vbox {
   \vskip 1.0cm
   \centering%
   \normalfont\normalsize\@author%
   }\vskip 1.5cm plus 0.5cm}%

% Redefine \section and \subsection:
\renewcommand\section{\@startsection {section}{1}{\z@}%
                                  {-2.5ex \@plus -0.2ex \@minus -.2ex}%
                                  {0.1ex \@plus .1ex}%
   {\normalfont\raggedright\bfseries\rmfamily\if@ugly\uppercase\fi}}
\renewcommand\subsection{\@startsection{subsection}{2}{\z@}%
                                    {-2.5ex\@plus -0.2ex \@minus -.2ex}%
                                    {0.1ex \@plus .1ex}%
   {\normalfont\raggedright\bfseries\rmfamily}}
%
\newenvironment{acknowledgements}{%
  \section*{\if@ugly ACKNOWLEDGEMENTS\else Acknowledgements\fi}}%
  {}%
%
\renewenvironment{abstract}{%
  \section*{\if@ugly ABSTRACT\else Abstract\fi}}%
  {}%
%
\newenvironment{biography}{%
  \section*{\if@ugly BIOGRAPHY\else Biography\fi}}%
  {}%
%

\def\refname{\if@ugly REFERENCES\else References\fi}
% Cover our bases in babel:
\addto\captionsamerican{%
  \renewcommand{\refname}%
   {\if@ugly REFERENCES\else References\fi}%
}
\addto\captionsenglish{%
  \renewcommand{\refname}%
   {\if@ugly REFERENCES\else References\fi}%
}

\long\def\@makecaption#1#2{%
 \vskip\abovecaptionskip
 \small
 \sbox\@tempboxa{\textbf{#1.} #2}%
 \ifdim \wd\@tempboxa >\hsize
   \@hangfrom{\textbf{#1.} }#2\par
 \else
   \global \@minipagefalse
   \hb@xt@\hsize{\hfil\box\@tempboxa\hfil}%
 \fi
 \vskip\belowcaptionskip
}%
\renewcommand\floatpagefraction{.9}
\renewcommand{\topfraction}{.9}
\renewcommand{\bottomfraction}{.9}
\renewcommand{\textfraction}{.1}
\pagestyle{empty}
\input{fleqn.clo}

% Page dimensions 

\if@afour
  \setlength{\textwidth}{180mm}
  \setlength{\topmargin}{-5.4mm}% (but add ??? on first page)
  \setlength{\textheight}{247mm}
  \setlength{\oddsidemargin}{-10.4mm}
  \setlength{\evensidemargin}{-10.4mm}
\else
  \setlength{\textwidth}{7in}
  \setlength{\topmargin}{-0.25in}% (but add 0.25in on first page)
  \setlength{\textheight}{9.25in}
  \setlength{\oddsidemargin}{-0.25in}
  \setlength{\evensidemargin}{-0.25in}
\fi
\setlength\columnsep{0.5in}
\setlength\footskip{0in}
\setlength\headheight{0in}
\setlength\headsep{0in}
%
\setlength\parskip{0.0pt}
\setlength\parindent{0.8cm}
%
\if@ugly\else\renewcommand{\baselinestretch}{1.1}\fi
\flushbottom
%
\endinput
%%
%%paper End of file `ion.cls'.
