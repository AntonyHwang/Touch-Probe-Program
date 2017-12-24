classdef TouchProbe<handle 
    properties
        gui;
        %setup vars
        node;
        self_ip;
        master_ip;
        %subpub vars
        init_sub;
        sub_camera;
        sub_fsr;
        sub_servo_x;
        sub_servo_y;
        sub_servo_z;
        sub_pos;
        pub_servo_x;
        pub_servo_y;
        pub_servo_z;
        pub_servo_pos;
        servo_x_msg;
        servo_y_msg;
        servo_z_msg;
        servo_pos_msg;
        %other
        x=[0:0.06:5];
        y;
        force_value;
        c_x;
        c_y;
        c_z;
        c_height;
        c_offset;
        data;
        force_reading=[];
        fig;
    end
    
    %all the topic names and types
    properties
        cam_topic='tp/pi_camera_rgb';
        fsr_topic='force_sns/ch0';
        servo_x_topic='servo_ctrl/s1';
        servo_y_topic='servo_ctrl/s2';
        servo_z_topic='servo_ctrl/s3';
        servo_pos_topic='servo_ctrl/pos';
        
        cam_msg_type='sensor_msgs/Image';
        fsr_msg_type='std_msgs/Float32';
        servo_msg_type='std_msgs/Float32';
        servo_pos_type='geometry_msgs/Point32';
        node_name='Touch_probe';
    end
    
    methods
        %Constructor
        function self=TouchProbe(gui,master_ip,self_ip)
            self.gui=gui;
            %Network setup
            self.master_ip=master_ip;
            self.self_ip=self_ip;
            self.set_env();  
            self.node = robotics.ros.Node(self.node_name);
            %subs setup:
            self.init_subs();
            %pub setup
            self.init_pubs();
        end
        
        function disp_cam(self)
            self.sub_camera.NewMessageFcn=@self.camera_callback;
        end
        %getting camera image and display onto the axes
        function camera_callback(self,sub,msg)
            img=readImage(msg);
            imshow(img,'Parent',self.gui.UICameraViewer);
        end
        
        function stop_cam_disp(self)
            if  isempty(self.fig)
                return
            end
            self.sub_camera.NewMessageFcn=[];%Syntax for deleting objects
            self.fig.delete();
            self.fig=[];
        end
        
        function disp_fsr(self)
            self.sub_fsr.NewMessageFcn=@self.fsr_callback;
        end
        
        function stop_fsr_disp(self)
            self.sub_fsr.NewMessageFcn=[];
        end
        %publish target position to the topic
        function set_point(self,x,y,z)
            self.servo_pos_msg.X=x;
            self.servo_pos_msg.Y=y;
            self.servo_pos_msg.Z=z;
            if x ~= -1
                self.c_x = x;
                self.gui.xEditField.Value = self.c_x;
            end
            if y ~= -1
                self.c_y = y;
                self.gui.yEditField.Value = self.c_y;
            end
            if z ~= -1
                self.c_z = z;
                self.gui.zEditField.Value = self.c_z;
            end
            self.pub_servo_pos.send(self.servo_pos_msg);
        end
        
        %METHOD 1 BY FORCE
        function img = force_scan(self, step)
            step = 0.1;
            idx = 1 / step;
            self.data = zeros(7, 5, 3);
            row = 0;
            y_max = 3.5;
            x_max = 2.5;
            for y = 0:step:y_max
                if mod(row,2) == 0
                    for x = 0:step:x_max
                        sensor_data = [];
                        y_idx = int8((y_max - y) * idx + 1);
                        x_idx = int8(x * idx + 1);
                        self.set_point(x, y, 1.5);
                        pause(2);
                        self.set_point(-1, -1, 0);
                        pause(0.5);
                        self.data(y_idx, x_idx, :) = self.force_value;
                        self.data(y_idx, x_idx, :)
                        self.gui.zEditField.Value = 0;
                    end
                else
                    for x = x_max:-step:0
                        sensor_data = [];
                        y_idx = int8((y_max - y) * idx + 1);
                        x_idx = int8(x * idx + 1);
                        self.set_point(x, y, 1.5);
                        pause(2);
                        self.set_point(-1, -1, 0);
                        pause(0.5);
                        self.data(y_idx, x_idx, :) = self.force_value;
                        self.data(y_idx, x_idx, :)
                        self.gui.zEditField.Value = 0;
                    end
                end  
                row = row + 1;
            end
            img = self.get_3d_map(self.data);
        end
        
        function img = get_3d_map(self)
            max_val = max(self.data);
            min_val = min(self.data);
            rgb_ratio = 1 / (max_val - min_val);
            self.data = 1 - ((self.data - min_val) .* rgb_ratio);
            img = image(self.data)
        end
        
        %METHOD 2 DETECT HEIGHT BY BINARY SEARCH
        function img = z_scan(self, step)
            step = 0.1;
            idx = 1 / step;
            self.data = zeros(7, 5, 3);
            row = 0;
            y_max = 3.5
            x_max = 2.5
            for y = 0:step:y_max
                if mod(row,2) == 0
                    for x = 0:step:x_max
                        self.set_point(x, y, 1.5);
                        y_idx = int8((y_max - y) * idx + 1);
                        x_idx = int8(x * idx + 1);
                        pause(1)
                        self.data(y_idx, x_idx, :) = self.get_z() * 100;
                    end
                else
                    for x = x_max:-step:0
                        self.set_point(x, y, 1.5);
                        y_idx = int8((y_max - y) * idx + 1);
                        x_idx = int8(x * idx + 1);
                        pause(1)
                        self.data(y_idx, x_idx, :) = self.get_z() * 100;
                    end
                end
                row = row + 1;
            end
            img = self.get_3d_map(self.data);
            self.set_point(0, 0, 1.5);
        end
        
        function z = get_z(self)
            self.gui.zEditField.Value = 0;
            low_z = 0;
            high_z = 1.5;
            mid_z = 0;
            while high_z >= (low_z + 0.05)
                mid_z = (high_z + low_z) / 2.0;
                self.set_point(-1, -1, mid_z);
                pause(2);
                if self.force_value > 0.1
                    mid_z = (high_z + mid_z) / 2.0;
                    low_z = mid_z;
                elseif self.force_value < 0.1
                    mid_z = (mid_z + low_z) / 2.0;
                    high_z = mid_z;
                else
                    break
                end
            end
            z = mid_z;
            self.gui.HeightzEditField.Value = z;
        end
        
        %METHOD 3 BY CHANGE IN FORCE BACKTRACK
        function img = backtrack_scan(self, step)
            step = 0.1;
            idx = 1 / step;
            self.data = zeros(7, 5, 3);
            row = 0;
            y_max = 3.5
            x_max = 2.5
            for y = 0:step:y_max
                if mod(row,2) == 0
                    for x = 0:step:x_max
                        self.set_point(x, y, 1.5);
                        y_idx = int8((y_max - y) * idx + 1);
                        x_idx = int8(x * idx + 1);
                        pause(2);
                        self.data(y_idx, x_idx, :) = self.bt_get_z();
                        pause(2);
                    end
                else
                    for x = x_max:-step:0
                        self.set_point(x, y, 1.5);
                        y_idx = int8((y_max - y) * idx + 1);
                        x_idx = int8(x * idx + 1);
                        pause(2);
                        self.data(y_idx, x_idx, :) = self.bt_get_z();
                        pause(2);
                    end
                end
                row = row + 1;
            end
            img = self.get_3d_map();
            self.set_point(0, 0, 1.5);
        end
        
        function z = bt_get_z(self)
            z = 0;
            self.set_point(-1, -1, z);
            pause(3);
            %backtrack
            while self.force_value >= 0.000001
                z = z + 0.05;
                self.set_point(-1, -1, z);
                pause(2);
            end
            pause(1);
            self.set_point(-1, -1, 1.5);
            self.gui.HeightzEditField.Value = z + 1;
        end
        
        %METHOOD 4 BY DRAFT
        function img = draft_scan(self, step)
            step = 0.1;
            idx = 1 / step;
            self.data = zeros(7, 5, 3);
            y_max = 3.5
            x_max = 2.5
            for x = 0:step:x_max
                %reset position
                y = 0;
                x_idx = int8(x + 1);
                self.set_point(x, y, 1.5);
                pause(3);
                while y <= (y_max + step)
                    self.set_point(x, y, 0);
                    pause(0.5);
                    y_idx = int8((y_max - y) * idx + 1);
                    self.data(y_idx, x_idx, :) = self.force_value;
                    y = y + step;
                end
            end
            img = self.get_3d_map();
            self.set_point(0, 0, 1.5);
        end
        
        function set_env(self)
            sprintf("Setting ROS env variables:\n ROS_MASTER_URI =%s\nROS_IP =%s\n ",self.master_ip,self.self_ip);
            setenv('ROS_IP',self.self_ip);
            rosinit;
        end
        %initiate all the subscribers
        function init_subs(self)             
            self.sub_camera=rossubscriber(self.cam_topic,self.cam_msg_type);
            self.sub_fsr=rossubscriber(self.fsr_topic,self.fsr_msg_type);
            self.sub_servo_x =rossubscriber(self.servo_x_topic,self.servo_msg_type);
            self.sub_servo_y =rossubscriber(self.servo_y_topic,self.servo_msg_type);
            self.sub_servo_z =rossubscriber(self.servo_z_topic,self.servo_msg_type);
            self.sub_pos=rossubscriber(self.servo_pos_topic,self.servo_pos_type);
        end
        %initiate all the publishers
        function init_pubs(self)
            self.pub_servo_x=rospublisher(self.servo_x_topic); 
            self.pub_servo_y=rospublisher(self.servo_y_topic); 
            self.pub_servo_z=rospublisher(self.servo_z_topic); 
            self.pub_servo_pos=rospublisher(self.servo_pos_topic); 
            
            self.servo_x_msg=rosmessage(self.pub_servo_x);
            self.servo_y_msg=rosmessage(self.pub_servo_y);
            self.servo_z_msg=rosmessage(self.pub_servo_z);
            self.servo_pos_msg=rosmessage(self.pub_servo_pos);
        end
        
        function fsr_callback(self,sub,msg)
            v = double(msg.Data) / 1000;
            %equation of trend line obtained from excel
            if v >= 1.49
            %force in Newton
                force = (1023.32*v^4 - 8134.80*v^3 + 24238.31*v^2 - 31803.26*v + 15466.20) * 0.0098;
            else
                force = (v * 20 / 1.49) * 0.0098;
            end
            
            self.force_value = force;
            
            if self.c_z == 1.5
                self.force_value = 0;
                self.c_offset = force;
                self.gui.OffsetNEditField.Value = force;
                self.gui.ForceNEditField.Value = self.force_value;
            else
                self.gui.ForceNEditField.Value = self.force_value;
                self.c_height = 3932.5 * self.force_value^2 - 175.46 * self.force_value + 1.9324;

                if self.c_height < 0
                    self.c_height = 0;
                end
                self.gui.HeightmmEditField.Value = self.c_height;

            end
%              self.force_value = force;
%              if self.force_value < 0
%                 self.force_value = 0;
%              end
%              if self.c_z == 1.5 || self.force_value < 0.22
%                 self.gui.ForceNEditField.Value = force;
%                 self.gui.HeightmmEditField.Value = 0;
%              else
%                 self.gui.ForceNEditField.Value = self.force_value;
%                 self.c_height = 0.0799 * self.force_value^2 + 0.01 * self.force_value + 0.2308;
%                 self.gui.HeightmmEditField.Value = self.c_height;
%              end
             %self.c_height = 0.122 * self.force_value^2 - 0.001 * self.force_value + 0.14;
        
%             self.force_value = force;
%             if self.force_value < 0
%                 self.force_value = 0;
%             end
%             self.gui.ForceNEditField.Value = self.force_value;
        end
    end
end

