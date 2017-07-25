function plotConfMat(desired,computed)
  confMat = confMatGet(desired, computed);
  opt=confMatPlot('defaultOpt');
  opt.className={'Inliers', 'Outliers'};
  opt.mode='percentage';
  opt.format='8.2f';
  figure; confMatPlot(confMat, opt);
end
