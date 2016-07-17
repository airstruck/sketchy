local lg, lm = love.graphics, love.math

-- shape types

local shapeMethodByType = {
    chain = 'drawChainShape',
    circle = 'drawCircleShape',
    edge = 'drawEdgeShape',
    polygon = 'drawPolygonShape',
}

local function drawCircleShape (self, shape, fixture)
    local body = fixture:getBody()
    local x, y = body:getWorldPoint(shape:getPoint())
    local r = shape:getRadius()
    lg.circle('fill', x, y, r, math.max(10, r * self.scale))
end

local function drawPolygonShape (self, shape, fixture)
    local body = fixture:getBody()
    lg.polygon('fill', body:getWorldPoints(shape:getPoints()))
end

local function drawChainShape (self, shape, fixture)
    local body = fixture:getBody()
    lg.polygon('line', body:getWorldPoints(shape:getPoints()))
end

local function drawEdgeShape (self, shape, fixture)
    local body = fixture:getBody()
    lg.line(body:getWorldPoints(shape:getPoints()))
end

-- joint types

local jointMethodByType = {
    distance = 'drawDistanceJoint', 
    friction = 'drawFrictionJoint',
    gear = 'drawGearJoint',
    motor = 'drawMotorJoint',
    mouse = 'drawMouseJoint',
    prismatic = 'drawPrismaticJoint',
    pulley = 'drawPulleyJoint',
    revolute = 'drawRevoluteJoint',
    rope = 'drawRopeJoint',
    weld = 'drawWeldJoint',
    wheel = 'drawWheelJoint',
}

local function drawDistanceJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.distanceJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawFrictionJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.frictionJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawGearJoint (self, joint)
    -- not yet implemented
end

local function drawMotorJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.motorJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawMouseJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.mouseJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawPrismaticJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.prismaticJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawPulleyJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    local gx1, gy1, gx2, gy2 = joint:getGroundAnchors( )
    lg.setColor(self.pulleyJointColor)
    lg.line(x1, y1, gx1, gy1)
    lg.line(gx1, gy1, gx2, gy2)
    lg.line(gx2, gy2, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawRevoluteJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.revoluteJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawRopeJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.ropeJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawWeldJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.weldJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawWheelJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(self.wheelJointColor)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

-- base types

local function drawBodyJoints (self, body)
    local joints = body:getJointList()
    for i = 1, #joints do
        self:presentJoint(joints[i])
    end
end

local function drawBodyContacts (self, body)
    local contacts = body:getContactList()
    lg.setColor(self.contactIndicatorColor)
    for i = 1, #contacts do
        self:drawContact(contacts[i])
    end
end

local function drawBodyAngle (self, body)
    lg.setColor(self.angleIndicatorColor)
    local angle = body:getAngle()
    local size = self.angleIndicatorSize / self.scale
    local vx = math.cos(angle) * size
    local vy = math.sin(angle) * size
    local sharp = self.angleIndicatorSharpness
    local x, y = body:getPosition()
    lg.polygon('fill',
        x - vy / sharp, y + vx / sharp,
        x + vx, y + vy,
        x + vy / sharp, y - vx / sharp)
end

local function drawContact (self, contact)
    if self.drawnObjects[contact] then return end
    self.drawnObjects[contact] = true
    local x1, y1, x2, y2 = contact:getPositions()
    local size = self.contactIndicatorSize / self.scale
    local segments = self.contactIndicatorSegments
    if x1 then
        lg.circle('fill', x1, y1, size, segments)
    end
    if x2 then
        lg.circle('fill', x2, y2, size, segments)
    end
end

local function drawFixture (self, fixture)
    local body = fixture:getBody()
    local bodyType = body:getType()
    if bodyType == 'static' then
        lg.setColor(self.staticFixtureColor)
    elseif bodyType == 'kinematic' then
        lg.setColor(self.kinematicFixtureColor)
    elseif body:isAwake() then
        lg.setColor(self.dynamicFixtureColor)
    else
        lg.setColor(self.sleepingFixtureColor)
    end
    self:drawShape(fixture:getShape(), fixture)
end

local function drawShape (self, shape, fixture)
    local f = self[shapeMethodByType[shape:getType()]]
    if f then
        f(self, shape, fixture)
    end
end

local function drawJoint (self, joint)
    local f = self[jointMethodByType[joint:getType()]]
    if f then
        f(self, joint)
    end
end

local function presentBody (self, body)
    -- presenter
    local present = self:getPresenter(body)
    if present then
        local finished = present(body)
        if finished then return finished end
    end
    
    self:drawBodyJoints(body)
    self:drawBodyContacts(body)
    self:drawBodyAngle(body)
end

local function presentFixture (self, fixture)
    local present = self:getPresenter(fixture)
    if present then
        local finished = present(fixture)
        if finished then return finished end
    end
    self:drawFixture(fixture)
end

local function presentJoint (self, joint)
    if self.drawnObjects[joint] then return end
    self.drawnObjects[joint] = true
    
    local present = self:getPresenter(joint)
    if present then
        local finished = present(joint)
        if finished then return finished end
    end
    
    self:drawJoint(joint)
end

local function screenToWorld (self, px, py)
    local x, y = self.left, self.top
    local w, h, s = self.width * 0.5, self.height * 0.5, self.scale
    local cx, cy = self.cameraX, self.cameraY
    local sin, cos = math.sin(-self.angle), math.cos(-self.angle)
    
    px, py = px - (x + w), py - (y + h)
    px, py = cos * px - sin * py, sin * px + cos * py
    px, py = px + cx * s, py + cy * s
    px, py = px / s, py / s
    
    return px, py
end

local function worldToScreen (self, px, py)
    local x, y = self.left, self.top
    local w, h, s = self.width * 0.5, self.height * 0.5, self.scale
    local cx, cy = self.cameraX, self.cameraY
    local sin, cos = math.sin(-self.angle), math.cos(-self.angle)
    
    px, py = px * s, py * s
    px, py = px - cx * s, py - cy * s
    px, py = cos * px + sin * py, sin * -px + cos * py
    px, py = px + (x + w), py + (y + h)
    
    return px, py
end

-- Returns point components of the aabb
-- enclosing the circle circumscribing the input aabb.
local function expandAABB (x1, y1, x2, y2)
    local w, h = x2 - x1, y2 - y1
    local cx, cy = x1 + w * 0.5, y1 + h * 0.5
    local r = 0.5 * math.sqrt(w * w + h * h)
    return cx - r, cy - r, cx + r, cy + r
end

local function drawWorld (self, world)
    local x, y = self.left, self.top
    local w, h, s = self.width * 0.5, self.height * 0.5, self.scale
    local ws, hs = w / s, h / s
    local cx, cy = self.cameraX, self.cameraY
    
    lg.push('all')
    
    lg.setLineWidth(self.jointLineWidth / self.scale)
    lg.setPointSize(self.jointPointSize)
    
    -- translate center of viewport to origin; rotate to angle
    lg.translate(x + w, y + h)
    lg.rotate(self.angle)
	
    -- translate camera position to center of viewport; scale
    lg.translate(-cx * s, -cy * s)
    lg.scale(s, s)
    
    -- draw any fixtures intersected by viewport
    local relatedBodies = {}
    
    local function draw (fixture)
        local body = fixture:getBody()
        if not relatedBodies[body] then
            local i = #relatedBodies + 1
            relatedBodies[i] = body
            relatedBodies[body] = i
        end
        self:presentFixture(fixture)
        return true
    end
    
    local x1, y1, x2, y2 = expandAABB(cx - ws, cy - hs, cx + ws, cy + hs)
    world:queryBoundingBox(x1, y1, x2, y2, draw)
        
    -- draw bodies connected to the fixtures
    -- (joints, contacts, angle indicators)
    for i = 1, #relatedBodies do
        self:presentBody(relatedBodies[i])
    end
    
    -- paranoia
    relatedBodies = nil
    draw = nil
    
    lg.pop()
end

-- api

local function draw (self, thing)
    self.drawnObjects = {}
    if thing.getBodyList then
        return self:drawWorld(thing)
    end
    if thing.getFixtureList then
        return self:presentBody(thing)
    end
end

local function setAngle (self, angle)
    self.angle = angle
    return self
end

local function getAngle (self)
    return self.angle
end

local function setCamera (self, x, y)
    self.cameraX, self.cameraY = x or 0, y or 0
    return self
end

local function getCamera (self)
    return self.cameraX, self.cameraY
end

local function setViewport (self, left, top, width, height)
    self.left, self.top, self.width, self.height = left, top, width, height
    return self
end

local function getViewport (self)
    return self.left, self.top, self.width, self.height
end

local function setScale (self, scale)
    self.scale = scale
    return self
end

local function getScale (self)
    return self.scale
end

local function setPresenter (self, object, presenter)
    local data = object:getUserData()
    if data == nil then
        data = {}
        object:setUserData(data)
    end
    data.sketchyPresenter = presenter
    
    return self
end

local function getPresenter (self, object)
    local data = object:getUserData()
    if type(data) == 'table' then
        return data.sketchyPresenter
    end
end

return function ()
    return {
        -- internal
        drawnObjects = {},
        -- misc. colors and sizes
        angleIndicatorColor = { 255, 255, 255, 64 }, 
        angleIndicatorSize = 12,
        angleIndicatorSharpness = 3,
        contactIndicatorColor = { 255, 0, 0 }, 
        contactIndicatorSize = 4,
        contactIndicatorSegments = 8,
        jointLineWidth = 2,
        jointPointSize = 4,
        -- fixture colors
        staticFixtureColor = { 255, 0, 0, 128 },
        kinematicFixtureColor = { 128, 0, 128, 128 },
        dynamicFixtureColor = { 0, 0, 255, 128 },
        sleepingFixtureColor = { 0, 0, 128, 128 },
        -- joint colors
        distanceJointColor = { 0, 255, 0, 128 }, 
        frictionJointColor = { 255, 0, 128, 128 },
        gearJointColor = { 1, 1, 1, 128 },
        motorJointColor = { 0, 160, 160, 128 },
        mouseJointColor = { 0, 255, 255, 128 },
        prismaticJointColor = { 128, 192, 160, 128 },
        pulleyJointColor = { 0, 128, 255, 128 },
        revoluteJointColor = { 128, 255, 0, 128 },
        ropeJointColor = { 128, 128, 0, 128 },
        weldJointColor = { 255, 128, 0, 128 },
        wheelJointColor = { 128, 160, 128, 128 },
        -- initial values
        left = 0,
        top = 0,
        width = 800,
        height = 600,
        scale = 1,
        angle = 0,
        cameraX = 0,
        cameraY = 0,
        -- base types
        drawBodyAngle = drawBodyAngle,
        drawBodyContacts = drawBodyContacts,
        drawBodyJoints = drawBodyJoints,
        drawContact = drawContact,
        drawFixture = drawFixture,
        drawJoint = drawJoint,
        drawShape = drawShape,
        drawWorld = drawWorld,
        -- shape types
        drawChainShape = drawChainShape,
        drawCircleShape = drawCircleShape,
        drawEdgeShape = drawEdgeShape,
        drawPolygonShape = drawPolygonShape,
        -- joint types
        drawDistanceJoint = drawDistanceJoint,
        drawFrictionJoint = drawFrictionJoint,
        drawGearJoint = drawGearJoint,
        drawMotorJoint = drawMotorJoint,
        drawMouseJoint = drawMouseJoint,
        drawPrismaticJoint = drawPrismaticJoint,
        drawPulleyJoint = drawPulleyJoint,
        drawRevoluteJoint = drawRevoluteJoint,
        drawRopeJoint = drawRopeJoint,
        drawWeldJoint = drawWeldJoint,
        drawWheelJoint = drawWheelJoint,
        -- presenter hooks
        presentBody = presentBody,
        presentFixture = presentFixture,
        presentJoint = presentJoint,
        -- api
        draw = draw,
        setAngle = setAngle,
        getAngle = getAngle,
        setCamera = setCamera,
        getCamera = getCamera,
        setPresenter = setPresenter,
        getPresenter = getPresenter,
        setScale = setScale,
        getScale = getScale,
        setViewport = setViewport,
        getViewport = getViewport,
        screenToWorld = screenToWorld,
        worldToScreen = worldToScreen,
    }
end
