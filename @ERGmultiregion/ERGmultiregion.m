classdef ERGmultiregion
%ERGMULTIREGION Defines an union of ERGregions for an instance of
%ERGsys to operate in.
%
%   Constructor:
%       ERGMULTIREGION(reg_1,...,reg_n) - Creates an ERGmultiregion
%       object as the union of all arguments, which must be instances
%       of ERGregion or ERGmultiregion. An instance of ERGmultiregion
%       can also be created via the logical or operation between two or
%       more instances of ERGregion or ERGmultiregion.
%   Public methods:
%       or()
% 
%       rho(r,v,sys,eta)
% 
%       dist(reg1,reg2);
% 
%       checkIntersection(reg1,reg2);
    
    properties (SetAccess=immutable)
        regions
        intsc
    end
    
    properties (SetAccess=protected)
        C
    end
    
    methods
        function this=ERGmultiregion(varargin)
            regions={};
            intsc={};
            C=0;
            for i=1:nargin %For all items in varargin (may contain ERGMultiRegions)
                sz=length(regions); %Size of the current output object list
                if isa(varargin{i},'ERGregion')
                    regiontobeadded=varargin{i};
                    for j=1:sz %For all items in the output object region list
                        regiontobechecked=regions{j};
                        if ERGmultiregion.checkIntersection(regiontobeadded,regiontobechecked) %If the intersection between them is nonempty
                            d=ERGmultiregion.dist(regiontobeadded,regiontobechecked);
                            C(j,sz+1)=d;
                            C(sz+1,j)=d;
                            int=Polyhedron([regiontobeadded.M;regiontobechecked.M],[regiontobeadded.b;regiontobechecked.b]);
                            int=int.computeVRep();
                            int=int.minVRep();
                            vs=mean(int.V)';
                            intsc{j,sz+1}=vs;
                            intsc{sz+1,j}=vs;
                        end
                    end
                    regions{end+1}=regiontobeadded; %Add the current ERGregion to the output object's region list
                    C(sz+1,sz+1)=0;
                elseif isa(varargin{i},'ERGmultiregion')
                    multiregiontobeadded=varargin{i};
                    sztba=length(multiregiontobeadded.regions);
                    C(sz+1:sz+sztba,sz+1:sz+sztba)=multiregiontobeadded.C;
                    intsc(sz+1:sz+sztba,sz+1:sz+sztba)=multiregiontobeadded.intsc;
                    for j=1:sz %For all items in the output object region list
                        regiontobechecked=regions{j};
                        for k=1:sztba %For all items in the to be added list
                            if ERGmultiregion.checkIntersection(regiontobechecked,multiregiontobeadded.regions{k}) %If they are connected
                                d=ERGmultiregion.dist(regiontobechecked,multiregiontobeadded.regions{k});
                                C(j,sz+k)=d;
                                C(sz+k,j)=d;
                                int=Polyhedron([regiontobechecked.M;multiregiontobeadded.regions{k}.M],[regiontobechecked.b;multiregiontobeadded.regions{k}.b]);
                                int=int.computeVRep;
                                int=int.minVRep;
                                vs=mean(int.V)';
                                intsc{j,sz+k}=vs;
                                intsc{sz+k,j}=vs;
                            end
                        end
                    end
                    for k=1:sztba %For all items in the to be added list
                        %Add to the regions list
                        regions{end+1}=multiregiontobeadded.regions{k};
                    end
                    
                else
                    error(['Argument number ' num2str(i) ' is not an istance of ERGregion or ERGmultiregion'])
                end
                
            end
            nregs=length(regions);
            dim_vector=zeros(1,nregs);
            for l=1:nregs
                dim_vector(l)=size(regions{l}.M,2);
            end
            if all(dim_vector==dim_vector(1))
                this.C=C;
                this.regions=regions;
                this.intsc=intsc;
            else
                error('The dimensionality of some of the regions is not consistent')
            end
        end
        
        %Or operator overload
        out=or(varargin)
        
        %Navigation field
        out=rho(this,r,v,x,sys,eta)
        
        %Check in which region the state is
        out=whichRegion(this,x,v)
        
        %Disp overload
        disp(this)
    end
    
    methods(Static)
        %Calculate distance between centroids
        out=dist(reg1,reg2);
        
        %Check if two regions intersect
        out=checkIntersection(reg1,reg2);
    end
end