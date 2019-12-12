class TopologicalLocaliser():
	def init(self, pip, pport, topomap, fake=False):
		connection_url = "tcp://" + pip + ":" + str(pport)
        app = qi.Application(
            ["TopologicalNavigation", "--qi-url=" + connection_url]
        )
        app.start()
        session = app.session
        self.memProxy = session.service("ALMemory")

        self._sub_navgaol = self.memProxy.subscriber(
            "TopologicalNav/Goal"
        )
        self._sub_navgaol.signal.connect(self._on_goal)


    def _on_goal(self, goal):
    	logger.info('waiting for current goal to be cancelled')
        while (self.goal_active):
            self.goal_reached = False
            self.cancelled = True
            sleep(.5)
        logger.info('it is cancelled')

        self.navigate(goal)

    def navigate(self, target):
    	o_node = get_node(self.map, self.closest_node) # <--
    	g_node = get_node(self.map, target) # <--

    	rsearch = TopologicalRouteSearch(self.map)
        route = rsearch.search_route(o_node.name, target) # <--

        self.follow_route(route)

    def follow_route(self, route):
		orig = route.source[0]

		o_node = get_node(self.map, orig)
        a = get_edge_from_id(self.map,
                             route.source[0],
                             route.edge_id[0]).action

        if self.current_node == 'none':
            logger.info('Do planner to %s' % (self.closest_node))
            self.current_target = orig
            nav_ok = self.monitored_navigation(o_node, 'NAOqiPlanner/Goal')

        rindex = 0
        nav_ok = True
        self.cancelled = False
        route_len = len(route.edge_id)

        while rindex < route_len and not self.cancelled:
            cedg = get_edge_from_id(
                self.map, route.source[rindex], route.edge_id[rindex]
            )
            cnode = get_node(self.map, cedg.node)

            a = cedg.action
            # next action
            if rindex < (route_len - 1):
                a1 = get_edge_from_id(
                    self.map,
                    route.source[rindex + 1],
                    route.edge_id[rindex + 1]).action
            else:
                a1 = 'none'

            self.current_action = a
            self.next_action = a1

            if a == a1:
                definite_action = "NAOqiPlanner/GoalXY"
            else:
                definite_action = a
            logger.info("From " + route.source[rindex] +
                        " do " + a + " to " + cedg.node)

            self.current_target = cedg.node
            nav_ok = self.monitored_navigation(cnode, definite_action)
            if nav_ok:
                rindex = rindex + 1


    def monitored_navigation(self, gnode, command):
        nav_ok = False
        gpose = gnode.pose
        self.goal_reached = False
        # big, big hack here: If the command ends in "XY",
        # then only send to X and Y coords
        if command.endswith('XY'):
            goal_pose = [gpose.position.x, gpose.position.y]
        else:
            goal_pose = [gpose.position.x,
                         gpose.position.y,
                         gpose.orientation.z]

        while ( # while 1
            nTry < self._max_retries and
            not self.goal_reached and
            not self.cancelled
        ):
            self.failure = False
            self.memProxy.raiseEvent(command, goal_pose)

            while ( # while 2
                not self.cancelled and
                not self.goal_reached and
                not self.failure
            ):
                # ONLY DO THIS IF XY ACTION
                if (
                    (self.current_node == gnode.name) and
                    command.endswith('XY')
                ):
                    logger.info(
                            "we are in reach of the goal, "
                            "so let's report success")
                    return True

                time.sleep(0.1)
            #end while 2

            if self.goal_reached:
                nav_ok = True
                logger.info("  succeeded going to %s" % gnode.name)
                self.memProxy.raiseEvent("TopologicalNav/Status",
                                         "PlannerSuccesful")
                return nav_ok

            elif self.failure:
                failmsg = "in base all'errore"
                self.memProxy.raiseEvent(
                            "TopologicalNav/Status",
                            failmsg)


        return nav_ok
    # end monitored navigation