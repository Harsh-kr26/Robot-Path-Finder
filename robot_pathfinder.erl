-module(robot_pathfinder).
-export([find_paths/2, start/3, init_db/0, store_grid_info/1, store_robot_info/3, store_path/2, get_all_data/0]).

%% @doc Type definitions
%% @end 
-type position() :: {integer(), integer()}.
-type path() :: [position()].
-type robot_id() :: integer().
-type grid_size() :: {pos_integer(), pos_integer()}.

%% @doc Define records for node
%% @end
-record(node, {
    position :: position(),
    g_cost = 0 :: non_neg_integer(),
    h_cost = 0 :: non_neg_integer(),
    f_cost = 0 :: non_neg_integer(),
    parent = undefined :: undefined | #node{}
}).

%% @doc Define Mnesia table records
%% @end
-record(grid_info, {key :: atom(), size :: grid_size()}).
-record(robot_info, {id :: robot_id(), start_pos :: position(), goal_pos :: position()}).
-record(robot_path, {id :: robot_id(), path :: path()}).

%% @doc Initialize Mnesia database
%% @end
-spec init_db() -> ok.
init_db() ->
    mnesia:create_schema([node()]),
    mnesia:start(),
    
    %% Create tables
    mnesia:create_table(grid_info, [{attributes, record_info(fields, grid_info)},
                                    {disc_copies, [node()]}]),
    mnesia:create_table(robot_info, [{attributes, record_info(fields, robot_info)},
                                     {disc_copies, [node()]}]),
    mnesia:create_table(robot_path, [{attributes, record_info(fields, robot_path)},
                                     {disc_copies, [node()]}]),
    
    mnesia:wait_for_tables([grid_info, robot_info, robot_path], 5000).

%% @doc Store grid size
%% @end
-spec store_grid_info(grid_size()) -> {atomic, ok} | {aborted, any()}.
store_grid_info(Size) ->
    Transaction = fun() ->
        mnesia:write(#grid_info{key = grid_size, size = Size})
    end,
    mnesia:transaction(Transaction).

%% @doc Store robot information
%% @end
-spec store_robot_info(robot_id(), position(), position()) -> {atomic, ok} | {aborted, any()}.
store_robot_info(RobotId, StartPos, GoalPos) ->
    Transaction = fun() ->
        mnesia:write(#robot_info{id = RobotId, start_pos = StartPos, goal_pos = GoalPos})
    end,
    mnesia:transaction(Transaction).

%% @doc Store robot path
%% @end
-spec store_path(robot_id(), path()) -> {atomic, ok} | {aborted, any()}.
store_path(RobotId, Path) ->
    Transaction = fun() ->
        mnesia:write(#robot_path{id = RobotId, path = Path})
    end,
    mnesia:transaction(Transaction).

%% @doc Retrieve all stored data
%% @end
-spec get_all_data() -> {[#grid_info{}], [#robot_info{}], [#robot_path{}]}.
get_all_data() ->
    %% Get grid info
    {atomic, GridInfo} = mnesia:transaction(fun() -> 
        mnesia:select(grid_info, [{'_', [], ['$_']}])
    end),
    
    %% Get robot info
    {atomic, RobotInfo} = mnesia:transaction(fun() -> 
        mnesia:select(robot_info, [{'_', [], ['$_']}])
    end),
    
    %% Get robot paths
    {atomic, RobotPaths} = mnesia:transaction(fun() -> 
        mnesia:select(robot_path, [{'_', [], ['$_']}])
    end),
    
    io:format("Grid Info: ~p~n", [GridInfo]),
    io:format("Robot Info: ~p~n", [RobotInfo]),
    io:format("Robot Paths: ~p~n", [RobotPaths]),
    
    {GridInfo, RobotInfo, RobotPaths}.
  
%% @doc Manhattan distance heuristic
%% @end
-spec manhattan_distance(position(), position()) -> non_neg_integer().
manhattan_distance({X1, Y1}, {X2, Y2}) ->
    abs(X1 - X2) + abs(Y1 - Y2).

%% @doc Check if a move is valid on the grid
%% @end
-spec is_valid_move(position()) -> boolean().
is_valid_move({X, Y}) ->
    X >= 1 andalso X =< 10 andalso 
    Y >= 1 andalso Y =< 10.

%% @doc Generate possible moves
%% @end
-spec generate_moves(position()) -> [position()].
generate_moves({X, Y}) ->
    [
        {X, Y},     %% Stay in current cell
        {X+1, Y},   %% Right
        {X-1, Y},   %% Left
        {X, Y+1},   %% Up
        {X, Y-1}    %% Down
    ].

%% @doc Reconstruct path from goal node
%% @end
-spec reconstruct_path(#node{}) -> path().
reconstruct_path(Node) ->
    reconstruct_path(Node, []).

-spec reconstruct_path(#node{}, path()) -> path().
reconstruct_path(#node{parent = undefined, position = Pos}, Acc) ->
    lists:reverse([Pos | Acc]);
reconstruct_path(#node{parent = Parent, position = Pos}, Acc) ->
    reconstruct_path(Parent, [Pos | Acc]).

%% @doc Find the node with lowest f_cost in open list
%% @end
-spec find_lowest_f_cost([#node{}], fun((#node{}, #node{}) -> boolean())) -> #node{}.
find_lowest_f_cost([Node], _) -> Node;
find_lowest_f_cost([Node1, Node2 | Rest], CompareF) ->
    LowestNode = case CompareF(Node1, Node2) of
        true -> Node1;
        false -> Node2
    end,
    find_lowest_f_cost([LowestNode | Rest], CompareF).

%% @doc Compare nodes by f_cost
%% @end
-spec compare_nodes(#node{}, #node{}) -> boolean().
compare_nodes(#node{f_cost = F1}, #node{f_cost = F2}) -> F1 =< F2.

%% @doc A* algorithm implementation
%% @end
-spec a_star(position(), position(), [position()]) -> path() | {error, no_path}.
a_star(Start, Goal, Obstacles) ->
    %% Initialize first node
    StartNode = #node{
        position = Start, 
        g_cost = 0, 
        h_cost = manhattan_distance(Start, Goal),
        f_cost = manhattan_distance(Start, Goal)
    },
    
    %% Start path finding
    find_path([StartNode], [], Goal,Obstacles).

%% @doc Recursive path finding
%% @end
-spec find_path([#node{}], [position()], position(), [position()]) -> path() | {error, no_path}.
find_path([], _, _, _) -> 
    {error, no_path};
find_path(OpenList, ClosedList, Goal, Obstacles) ->
    
    CurrentNode = find_lowest_f_cost(OpenList, fun compare_nodes/2),
    
    
    NewOpenList = lists:delete(CurrentNode, OpenList),
    
    %% Check if we've reached the goal
    case CurrentNode#node.position of
        Goal -> 
            reconstruct_path(CurrentNode);
        _ ->
            %% Add current to closed list
            NewClosedList = [CurrentNode#node.position | ClosedList],
            
            
            Neighbors = lists:filter(
                fun(Move) -> 
                    is_valid_move(Move) andalso 
                    not lists:member(Move, NewClosedList) andalso
                    not lists:member(Move, Obstacles)
                end, 
                generate_moves(CurrentNode#node.position)
            ),
            
            %% Process neighbors
            {UpdatedOpenList, _} = lists:foldl(
                fun(Neighbor, {AccOpenList, Node}) ->
                    %% Calculate costs
                    GCost = Node#node.g_cost + 1,
                    HCost = manhattan_distance(Neighbor, Goal),
                    FCost = GCost + HCost,
                    
                    %% Create neighbor node
                    NeighborNode = #node{
                        position = Neighbor,
                        g_cost = GCost,
                        h_cost = HCost,
                        f_cost = FCost,
                        parent = Node
                    },
                    
                    %% Check if neighbor is already in open list
                    case lists:keyfind(Neighbor, #node.position, AccOpenList) of
                        #node{f_cost = OldFCost} when OldFCost =< FCost ->
                            {AccOpenList, Node};
                        _ ->
                            %% Add to open list
                            {[NeighborNode | AccOpenList], Node}
                    end
                end,
                {NewOpenList, CurrentNode},
                Neighbors
            ),
            
            %% Continue searching
            find_path(UpdatedOpenList, NewClosedList, Goal, Obstacles)
    end.

-spec find_paths([position()], [position()]) -> [path()] | {error, {no_path_for_robot, position(), position()}}.
find_paths(RobotStarts, RobotGoals) ->
    Orderings = permutations(lists:seq(1, length(RobotStarts))),
    try_orders(Orderings, RobotStarts, RobotGoals).
%% @doc Try all possible orders
%% @end
-spec try_orders([[integer()]], [position()], [position()]) -> [path()] | {error, no_path_found}.
try_orders([], _RobotStarts, RobotGoals) ->
    {error, no_path_found};
try_orders([Order|RestOrders], RobotStarts, RobotGoals) ->
    case plan_paths(Order, RobotStarts, RobotGoals) of
        {error, _} ->
            try_orders(RestOrders, RobotStarts, RobotGoals);
        Paths ->
            case paths_intersect(Paths) of
                true ->
                    try_orders(RestOrders, RobotStarts, RobotGoals);
                false ->
                    Paths
            end
    end.
%% @doc Plan a path considering a orders
%% @end

-spec plan_paths([integer()], [position()], [position()]) -> [path()] | {error, {no_path_for_robot, position(), position()}}.
plan_paths(Order, RobotStarts, RobotGoals) ->
    plan_paths(Order, RobotStarts, RobotGoals, []).


-spec plan_paths([integer()], [position()], [position()], [path()]) -> [path()] | {error, {no_path_for_robot, position(), position()}}.
plan_paths([], _RobotStarts, _RobotGoals, Paths) ->
    lists:reverse(Paths);
plan_paths([Index|Rest], RobotStarts, RobotGoals, Paths) ->
    Start = lists:nth(Index, RobotStarts),
    Goal = lists:nth(Index, RobotGoals),
    Obstacles = reserved_obstacles(Paths),
    case a_star(Start, Goal, Obstacles) of
        {error, no_path} ->
            {error, {no_path_for_robot, Start, Goal}};
        Path ->
            plan_paths(Rest, RobotStarts, RobotGoals, [Path|Paths])
    end.

%% @doc Reserve cell which are occupied in a path
%% @end
-spec reserved_obstacles([path()]) -> [position()].
reserved_obstacles(Paths) ->
    lists:foldl(fun(Path, Acc) ->
        Acc ++ tl(Path) ++ [lists:last(Path)]
    end, [], Paths).

%% @doc Check if path intersects
%% @end
-spec paths_intersect([path()]) -> boolean().
paths_intersect(Paths) ->
    AllPositions = lists:foldl(fun(Path, Acc) -> Acc ++ Path end, [], Paths),
    UniquePositions = lists:usort(AllPositions),
    length(AllPositions) =/= length(UniquePositions).

%% @doc Generate permutaions of orders
%% @end
-spec permutations([integer()]) -> [[integer()]].
permutations([]) -> [[]];
permutations(List) ->
    [[Elem|Rest] || Elem <- List, Rest <- permutations(lists:delete(Elem, List))].

%% @doc Example usage with database storage
%% @end
-spec start(pos_integer(), [position()], [position()]) -> {[#grid_info{}], [#robot_info{}], [#robot_path{}]} | {error, any()}.
start(N, Initial, Final) ->
    %% Initialize database
    init_db(),
    
    %% Store grid size
    store_grid_info({N, N}),
    
    
    %% Store robot IDs, start and goal positions
    lists:foreach(
        fun({Id, Start, Goal}) ->
            store_robot_info(Id, Start, Goal)
        end,
        lists:zip3(lists:seq(1, length(Initial)), Initial, Final)
    ),
    
    %% Find paths for robots
    case find_paths(Initial, Final) of
        {error, {no_path_for_robot, Start, Goal}} ->
            io:format("No path found for robot from ~p to ~p", [Start, Goal]),
            {error, {no_path_for_robot, Start, Goal}};
        {error, no_path_found} ->
            io:format("No non-intersecting paths found~n", []),
            {error, no_path_found};
        Paths ->
            %% Store and print paths
            lists:foreach(
                fun({Id, StartPos, Path}) ->
                    %% Store path in database
                    store_path(Id, Path)
                    %%io:format("Robot ~p from ~p path: ~p~n", [Id, StartPos, Path])
                end,
                lists:zip3(lists:seq(1, length(Initial)), Initial, Paths)
            ),


            
            %% Display all stored data
            get_all_data()
    end.
