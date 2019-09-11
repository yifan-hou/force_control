frc_data = dlmread('force_control_data.txt');
% egm_data = dlmread('../data/egm_data.txt');
% ati_data = dlmread('../data/netft_data.txt');

frc.time         = frc_data(:, 1);
frc.pose_set     = frc_data(:, 2:8);
frc.pose_fb      = frc_data(:, 9:15);
frc.wrench_fb    = frc_data(:, 16:21);
frc.wrench_Tr_all = frc_data(:, 22:27);
% frc.c1x          = frc_data(:, 22:24);
% frc.c1y          = frc_data(:, 25:27);
% frc.c2y          = frc_data(:, 28:30);
frc.pose_command = frc_data(:, 28:34);

starttime = frc.time(1);
frc.time  = frc.time - starttime;

% id = frc.time < 500;
% 
% frc.time = frc.time(id,:);
% frc.pose_set     = frc.pose_set(id,:);
% frc.pose_fb      = frc.pose_fb(id,:);
% frc.wrench_fb    = frc.wrench_fb(id,:);
% frc.c1x          = frc.c1x(id,:);
% frc.c1y          = frc.c1y(id,:);
% frc.c2y          = frc.c2y(id,:);
% frc.pose_command = frc.pose_command(id,:);


namelist = {'X', 'Y', 'Z'};
for i = 1:3
	figure(i);clf(i);hold on;
	subplot(2,1,1); hold on;
	title(['Pose ' namelist{i}]);
	plot(frc.time, frc.pose_set(:, i), '. -b', 'linewidth', 1.5);
	plot(frc.time, frc.pose_fb(:, i), '. -g', 'linewidth', 1.5);
	plot(frc.time, frc.pose_command(:, i), '. -r', 'linewidth', 1.5);
	legend('set', 'feedback', 'command');

	subplot(2,1,2); hold on;
	title(['Force ' namelist{i}]);
	plot(frc.time, frc.wrench_fb(:, i), '. ', 'linewidth', 1.5);
	legend('feedback');
end

% figure(4);clf(4);hold on;
% title('Quaternion');
% for i = 4:7
% 	subplot(4,1,i-3); hold on;
% 	plot(frc.time, frc.pose_set(:, i), '* - b', 'linewidth', 1.5);
% 	plot(frc.time, frc.pose_fb(:, i), '. - g', 'linewidth', 1.5);
% 	plot(frc.time, frc.pose_command(:, i), '. - r', 'linewidth', 1.5);
% 	legend('set', 'feedback', 'command');
% end
