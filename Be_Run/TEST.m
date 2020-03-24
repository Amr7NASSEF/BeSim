
uncertainty=[0.95, 1.05, 1];% percentage - randn(model.pred.nx,model.pred.nx) * model.pred.Ad
% for i=1:3
%     for row=1:model.uncert.nx
%         for col=1:model.uncert.nx
%            % cmd= ['A{',num2str(i),',',num2str(1),'}',  '(', num2str(row), ',' num2str(col), ') =' num2str(model.pred.Ad(row,col)*Precentage(i))];
%             model.uncert.AAd{i,1}(row,col)=model.pred.Ad(row,col)*uncertainty(i);
%             %=eval(cmd);
%         end
%     end
% end
A=ones(4,4)


for i=1:3
    RR{i,1}=A*uncertainty(i);
    
end

