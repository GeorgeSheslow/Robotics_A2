classdef TextToTraj < handle
    properties(Access =public)
        word;

        x_scale = 1;
        downsample_factor = 3;
        im_size = 500;
        
        % default offset values
        x_offset = -0.27;
        y_offset = -0.5;
        z_draw = 0.04;
        z_move = 0.06; 
        
        trajectory;
    end
    methods (Access =public)
        function [self] = TextToTraj(word) % input word to draw and x,y,z offsets
            self.word = word;
        end
        function setTextOffsets(self,offsets)
            disp("adding text offsets")
            self.x_offset = offset(1);
            self.y_offset = offsets(2);
            self.z_draw = offsets(3);
            self.z_move = self.z_draw + 0.1;
        end
        function addBaseOffsets(self,offsets)
            self.x_offset = self.x_offset + offsets(1);
            self.y_offset = self.y_offset + offsets(2);
            self.z_draw = self.z_draw + offsets(3);
            self.z_move = self.z_draw + 0.05;
        end
        function traj = GetTraj(self)
            traj = self.CalculateTraj();
            self.trajectory = traj;
        end
        function PlotTraj(self)
            plotData = self.trajectory;
            for i = 1:size(plotData,2)
                plot3(plotData(1,i),plotData(2,i),plotData(3,i),'k.');
                hold on
            end
        end
        function [z] = getDrawingHeight(self)
            z = self.z_draw;
        end
    end
     methods (Access =private)
        function trajectory = CalculateTraj(self)
            % select font based off OS
            if ismac
                font = 'SFNSRounded';
            elseif ispc
                font = "Ariel";
            elseif isunix
                font = 'SFNSRounded';
            else
                disp("Font not selected, for text")
            end
            image = 255 * ones(500,500,'uint8');
            image = insertText(image,[250,250],self.word,'AnchorPoint','Center','Font',font,'FontSize',35,'BoxColor','white');

            img = TextToTraj.skeletonize(image);
            img = imrotate(img,90);
            % imshow(img)
            grid_paths = TextToTraj.pathsOnGrid(img);
            grid_paths = TextToTraj.trimPath(grid_paths,2); %grid_paths trimmed with min path size
            
            coord_paths = TextToTraj.toCoordinates(grid_paths,img,self.x_scale);
            
            for i = 1:size(coord_paths,2) 
                x_interp = TextToTraj.downsampleInterp(coord_paths{i}(1,:),self.downsample_factor);
                y_interp = TextToTraj.downsampleInterp(coord_paths{i}(2,:),self.downsample_factor);
                coord_paths{i} = [x_interp ; y_interp];
            end

            trajectory = TextToTraj.stitchPath(coord_paths,self.x_offset,self.y_offset,self.z_move,self.z_draw);
        end

    end

%% Helper functions
    methods(Static)
        function r = binaryEmpty(img)
            r = 1;
            for i = 1:size(img,1)
                for j = 1:size(img,2)
                    if img(i,j) == 1
                        r = 0;
                        return
                    end
                end
            end
        end
        function image_skel = skeletonize(image_src)
            imageBW = im2bw(image_src); 
            image_skel = bwskel(imcomplement(imageBW)); 
        end
    
        function interp_coord = downsampleInterp(data, downsample_factor)
            assert(downsample_factor > 1, "downsample factor needs to be greater than 1")
            ti = linspace(0,1,length(data));
            tf = linspace(0,1, length(data) / downsample_factor);
            interp_coord = interp1(ti,data,tf,'pchip');  
        end
        function new_gridpoint = findRandomNeighbour(curr_row, curr_col, img)
            new_gridpoint = zeros(2,1);
            neighbours = [];
            for i = -1:1
                for j = -1:1
                    if img(curr_row+i, curr_col+j) == 1
                        neighbours = [neighbours [curr_row+i ; curr_col+j]];
                    end 
                end
            end
            
            random_index = randi([1 size(neighbours,2)]);
            new_gridpoint(1) = neighbours(1,random_index);
            new_gridpoint(2) = neighbours(2,random_index);
        end
        function new_gridpoint = findSingleNeighbour(curr_row, curr_col, img)
        
            new_gridpoint = zeros(2,1);
            
            for i = -1:1
                for j = -1:1
                    if img(curr_row+i, curr_col+j) == 1
                        new_gridpoint(1) = curr_row + i;
                        new_gridpoint(2) = curr_col + j;
                        return
                    end 
                end
            end
        end

        function [row_index, column_index] = findStart(img)
            %search inward from the corners
            for i = 1:size(img,1)-1
                for j = 1:size(img,2)-1
                    
                    if img(i, j) == 1
                        row_index = i;
                        column_index = j;
                        return
                    end
                    
                    if img(size(img,1) - i, j) == 1
                        row_index = size(img,1) - i;
                        column_index = j;
                        return
                    end
                    
                    if img(i, size(img,2) - j) == 1
                        row_index = i;
                        column_index = size(img,2) - j;
                        return
                    end
                    
                    if img(size(img,1) - i, size(img,2) - j) == 1
                        row_index = size(img,1) - i;
                        column_index = size(img,2) - j;
                        return
                    end
                    
                end
            end
        end

        function result = hasNeighbours(curr_row, curr_col, img)

            kernel = [1 1 1; 1 0 1; 1 1 1];
            tmp = kernel.*img(curr_row-1:curr_row+1,curr_col-1:curr_col+1); 
            result = sum(sum(tmp));
    
        end
        
        function paths = pathsOnGrid(binary_img)
        
            temp_img = binary_img;
            
            paths = {};
            
            while ~TextToTraj.binaryEmpty(temp_img)
                
                [start_row, start_col] = TextToTraj.findStart(temp_img);
                new_path = [start_row ; start_col];
                temp_img(start_row,start_col) = 0;
                
                % assumes that all pixels of interest are sufficently far from the
                % edge of the image
                
                num_neighbours = TextToTraj.hasNeighbours(new_path(1,end), new_path(2,end), temp_img);
                
                while num_neighbours ~= 0
                    % if only has a single neighbour
                    if num_neighbours == 1
                        new_point = TextToTraj.findSingleNeighbour(new_path(1,end), new_path(2,end), temp_img);
                        new_path = [new_path new_point];
                        num_neighbours = TextToTraj.hasNeighbours(new_path(1,end), new_path(2,end), temp_img);
                        temp_img(new_point(1),new_point(2)) = 0;
                        
                    else
                        %if multiple neighbours then right now just pick random
                        %neighbour
                        new_point = TextToTraj.findRandomNeighbour(new_path(1,end), new_path(2,end), temp_img);
                        new_path = [new_path new_point];
                        num_neighbours = TextToTraj.hasNeighbours(new_path(1,end), new_path(2,end), temp_img);
                        temp_img(new_point(1),new_point(2)) = 0;
                    end
                end
                
                paths{end+1} = new_path;
                
            end
        end
        function trajectory = stitchPath(coord_path, x_offset, y_offset, z_move, z_draw)
            trajectory = [];
            for path = 1:size(coord_path,2)  
                trajectory = [trajectory [coord_path{path}(:,1) + [x_offset ; y_offset] ; z_move]];
                
                for col = 1:size(coord_path{path},2)    
                    trajectory = [trajectory [coord_path{path}(:,col) + [x_offset ; y_offset] ; z_draw]];
                end
                
                trajectory = [trajectory [coord_path{path}(:,end) + [x_offset ; y_offset] ; z_move]];
                
            end    
        end
        function path_coordinates = toCoordinates(paths, img, x_length)
            x_px = size(img,2);
            S = x_length / x_px;
            
            path_coordinates = {};
            
            for path = 1:size(paths,2)
                
                path_coord = [];
                
                for i = 1:size(paths{path},2)
                    
                    x = (paths{path}(2,i) - 0.5) * S;
                    y = (size(img,1) - paths{path}(1,i) + 0.5) * S;
                    path_coord = [path_coord [x ; y]];
                
                end
                
                path_coordinates{end+1} = path_coord; %%TODO preallocate
                
            end
        end

        function trimmed_path = trimPath(paths,min_length)
        
            trimmed_path = {};
            
            for i = 1:size(paths,2)
                if size(paths{i},2) > min_length
                    trimmed_path{end+1} = paths{i}; %%TODO preallocate
                end
            end
        end

    end
end