-module(robot_pathfinder_tests).
-include_lib("eunit/include/eunit.hrl").
-include_lib("stdlib/include/assert.hrl").

%% @doc Record definitions needed for tests
-record(grid_info, {key, size}).
-record(robot_info, {id, start_pos, goal_pos}).
-record(robot_path, {id, path}).

%% @doc Helper function to clean up mnesia before and after tests
%% @end
setup() ->
    %% Stop mnesia if it's running
    case mnesia:system_info(is_running) of
        yes -> mnesia:stop();
        no -> ok
    end,
    %% Delete schema
    mnesia:delete_schema([node()]),
    ok.

cleanup(_) ->
    case mnesia:system_info(is_running) of
        yes -> mnesia:stop();
        no -> ok
    end,
    mnesia:delete_schema([node()]),
    ok.

%% @doc Fixture for running tests
mnesia_test_() ->
    {setup,
     fun setup/0,
     fun cleanup/1,
     [
        {"Test mnesia initialization", fun test_init_db/0},
        {"Test grid information storage", fun test_grid_info/0},
        {"Test robot information storage", fun test_robot_info/0},
        {"Test get_all_data functionality", fun test_get_all_data/0}
     ]
    }.

%% @doc Test mnesia initialization
test_init_db() ->
    %% Initialize database
    Result = robot_pathfinder:init_db(),
    ?assertEqual(ok, Result),
    
    %% Check that tables exist and have correct properties
    ?assertEqual(true, lists:member(grid_info, mnesia:system_info(tables))),
    ?assertEqual(true, lists:member(robot_info, mnesia:system_info(tables))),
    ?assertEqual(true, lists:member(robot_path, mnesia:system_info(tables))),
    
    %% Check table properties
    ?assertEqual(disc_copies, mnesia:table_info(grid_info, storage_type)),
    ?assertEqual(disc_copies, mnesia:table_info(robot_info, storage_type)),
    ?assertEqual(disc_copies, mnesia:table_info(robot_path, storage_type)).

%% @doc Test grid information storage
test_grid_info() ->
    %% Initialize database
    robot_pathfinder:init_db(),
    
    %% Test grid size storage
    GridSize = {10, 10},
    Result = robot_pathfinder:store_grid_info(GridSize),
    ?assertEqual({atomic, ok}, Result),
    
    %% Verify stored data
    {atomic, GridInfo} = mnesia:transaction(fun() -> 
        mnesia:select(grid_info, [{'_', [], ['$_']}])
    end),
    
    ?assertMatch([#grid_info{key = grid_size, size = {10, 10}}], GridInfo).

%% @doc Test robot information storage
test_robot_info() ->
    %% Initialize database
    robot_pathfinder:init_db(),
    
    %% Store robot info
    RobotId = 1,
    StartPos = {1, 1},
    GoalPos = {5, 5},
    Result = robot_pathfinder:store_robot_info(RobotId, StartPos, GoalPos),
    ?assertEqual({atomic, ok}, Result),
    
    %% Verify stored data
    {atomic, RobotInfo} = mnesia:transaction(fun() -> 
        mnesia:select(robot_info, [{'_', [], ['$_']}])
    end),
    
    ?assertMatch([#robot_info{id = 1, start_pos = {1, 1}, goal_pos = {5, 5}}], RobotInfo),
    
    %% Store another robot to test multiple robots
    Result2 = robot_pathfinder:store_robot_info(2, {5, 5}, {9, 9}),
    ?assertEqual({atomic, ok}, Result2),
    
    {atomic, RobotInfo2} = mnesia:transaction(fun() -> 
        mnesia:select(robot_info, [{'_', [], ['$_']}])
    end),
    
    ?assertEqual(2, length(RobotInfo2)),
    
    %% Sort by ID to ensure consistent order for testing
    SortedRobotInfo = lists:sort(fun(A, B) -> A#robot_info.id =< B#robot_info.id end, RobotInfo2),
    
    ?assertMatch([
        #robot_info{id = 1, start_pos = {1, 1}, goal_pos = {5, 5}},
        #robot_info{id = 2, start_pos = {5, 5}, goal_pos = {9, 9}}
    ], SortedRobotInfo).

%% @doc Test get_all_data functionality
test_get_all_data() ->
    %% Initialize database and store test data
    robot_pathfinder:init_db(),
    
    %% Store grid info
    robot_pathfinder:store_grid_info({10, 10}),
    
    %% Store robot info
    robot_pathfinder:store_robot_info(1, {1, 1}, {5, 5}),
    robot_pathfinder:store_robot_info(2, {5, 5}, {9, 9}),
    
    %% Store paths
    Path1 = [{1,1}, {1,2}, {1,3}, {2,3}, {3,3}, {3,4}, {4,4}, {5,5}],
    Path2 = [{5,5}, {6,6}, {7,7}, {8,8}, {9,9}],
    robot_pathfinder:store_path(1, Path1),
    robot_pathfinder:store_path(2, Path2),
    
    %% Get all data and check results
    {GridInfo, RobotInfo, RobotPaths} = robot_pathfinder:get_all_data(),

    %% Check grid info
    ?assertMatch([{grid_info,grid_size,{10,10}}], GridInfo),
    
    %% Sort records for consistent testing
    SortedRobotInfo = lists:sort(fun(A, B) -> A#robot_info.id =< B#robot_info.id end, RobotInfo),
    SortedRobotPaths = lists:sort(fun(A, B) -> A#robot_path.id =< B#robot_path.id end, RobotPaths),
    
    %% Check robot info
    ?assertMatch([
        {robot_info,1,{1,1},{5,5}},
        {robot_info,2,{5,5},{9,9}}
    ],RobotInfo),
    
    %% Check robot paths
    ?assertMatch([
        #robot_path{id = 1, path = [{1,1}, {1,2}, {1,3}, {2,3}, {3,3}, {3,4}, {4,4}, {5,5}]},
        #robot_path{id = 2, path = [{5,5}, {6,6}, {7,7}, {8,8}, {9,9}]}
    ], SortedRobotPaths).
    