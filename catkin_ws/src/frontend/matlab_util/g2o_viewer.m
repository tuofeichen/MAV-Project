% Visualize Posegraph
close all;
readEdge();
readVertex();
figure (1);
% Node1 = Node1(130:160);
% Node2 = Node2(130:160);
scatter(Node1, Node2);
xlabel('Node 1'); ylabel('Node 2');title('existing edges');

figure (2)
plot(NodeId,x,NodeId,y,NodeId,z);
legend('x','y','z')
xlabel('Node number'); ylabel('Translation');title('Final Pose Estimate');



