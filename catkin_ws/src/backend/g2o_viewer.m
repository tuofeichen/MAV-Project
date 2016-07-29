% Visualize Posegraph
readEdge();
readVertex();
figure (1);
scatter(Node1, Node2);
xlabel('Node 1'); ylabel('Node 2');title('existing edges');

figure (2)
plot(NodeId,x,NodeId,y,NodeId,z);
legend('x','y','z')
xlabel('Node number'); ylabel('Translation');title('Final Pose Estimate');



