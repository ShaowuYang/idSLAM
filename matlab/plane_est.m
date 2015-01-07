 %P = dlmread('1313680468.435014425.csv');
 %P = dlmread('1313680152.091522626.csv');
 % do PCA
 [pc,score,latent,tsquare] = princomp(P);
 
 % sort according to estimated z
 P = sortrows(P,3);
 
 % eigenvector of the smallest eigenvalue is plane normal
 n = pc(:,3);
 
 % distance estimate for each point
 D = P*n;
 
 d = mean(D);
 
 % plane equation is: p'*n - d = 0
 %                    x*nx + y*ny + z*nz -d = 0
 %                    z = (d - x*nx - y*ny)/nz 
 
 
 z_exp = (d - P(:,1)*n(1) - P(:,2)*n(2) )/n(3);
 err_z = z_exp - P(:,3);
 
 
 binsize = 10000;
 nbins = size(err_z,1)/binsize;
 
 binned_z_exp = [];
 binned_z_err = [];
 
 for i = 1:nbins
     from = (i-1)*binsize+1;
     to = i*binsize;
     binned_z_exp(i) = mean(z_exp(from:to));
     binned_z_err(i) = std(err_z(from:to));
 end % for
 
 plot(binned_z_exp,binned_z_err);
