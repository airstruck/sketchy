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
    lg.circle('fill', x, y, shape:getRadius())
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
    mouse = 'drawMouseJoint',
    prismatic = 'drawPrismaticJoint',
    pulley = 'drawPulleyJoint',
    revolute = 'drawRevoluteJoint',
    rope = 'drawRopeJoint',
    weld = 'drawWeldJoint',
}

local function drawDistanceJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(0, 255, 0, 128)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawFrictionJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(255, 0, 128, 128)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawGearJoint (self, joint)
    -- not yet implemented
end

local function drawMouseJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(0, 255, 255, 128)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawPrismaticJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(0, 255, 128, 128)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawPulleyJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    local gx1, gy1, gx2, gy2 = joint:getGroundAnchors( )
    lg.setColor(0, 128, 255, 128)
    lg.line(x1, y1, gx1, gy1)
    lg.line(gx1, gy1, gx2, gy2)
    lg.line(gx2, gy2, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawRevoluteJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(128, 255, 0, 128)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawRopeJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(128, 128, 0, 128)
    lg.line(x1, y1, x2, y2)
    lg.points(x1, y1, x2, y2)
end

local function drawWeldJoint (self, joint)
    local x1, y1, x2, y2 = joint:getAnchors()
    lg.setColor(255, 128, 0, 128)
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
    lg.setColor(255, 0, 0)
    for i = 1, #contacts do
        self:drawContact(contacts[i])
    end
end

local function drawBodyAngle (self, body)
    lg.setColor(255, 255, 255, 64)
    local angle = body:getAngle()
    local vx = math.cos(angle) * self.fatness * 4
    local vy = math.sin(angle) * self.fatness * 4
    local x, y = body:getPosition()
    lg.polygon('fill',
        x - vy / 4, y + vx / 4,
        x + vx, y + vy,
        x + vy / 4, y - vx / 4)
end

local function drawContact (self, contact)
    if self.drawnObjects[contact] then return end
    self.drawnObjects[contact] = true
    
    local x1, y1, x2, y2 = contact:getPositions()
    if x1 then
        lg.circle('fill', x1, y1, self.fatness)
    end
    if x2 then
        lg.circle('fill', x2, y2, self.fatness)
    end
end

local function drawFixture (self, fixture)
    local x1, y1, x2, y2 = fixture:getBoundingBox()
    lg.setColor(255, 255, 255, 64)
    lg.rectangle('line', x1, y1, x2 - x1, y2 - y1)
    lg.setColor(0, 0, 255, 128)
    self:drawShape(fixture:getShape(), fixture)
end

local function drawShape (self, shape, fixture)
    local f = self[shapeMethodByType[shape:getType()]]
    if f then
        f(self, shape, fixture)
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
    
    -- presenter
    local present = self:getPresenter(joint)
    if present then
        local finished = present(joint)
        if finished then return finished end
    end
    
    local f = self[jointMethodByType[joint:getType()]]
    if f then
        f(self, joint)
    end
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
    
    lg.setLineWidth(self.fatness / self.scale)
    lg.setPointSize(2 * self.fatness)
    
    -- draw border around viewport
    lg.setColor(255, 255, 255, 64)
    lg.rectangle('line', x, y, w * 2, h * 2)
	-- lg.setScissor(x, y, w * 2, h * 2)
    
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
            self:presentFixture(fixture)
        end
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

local weakKeyMeta = { __mode = 'k' }

return function ()
    return {
        presenters = setmetatable({}, weakKeyMeta),
        -- default values
        left = 0,
        top = 0,
        width = 800,
        height = 600,
        scale = 1,
        angle = 0,
        cameraX = 0,
        cameraY = 0,
        fatness = 2,
        drawnObjects = {},
        -- base types
        drawBodyAngle = drawBodyAngle,
        drawBodyContacts = drawBodyContacts,
        drawBodyJoints = drawBodyJoints,
        drawContact = drawContact,
        drawFixture = drawFixture,
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
        drawMouseJoint = drawMouseJoint,
        drawPrismaticJoint = drawPrismaticJoint,
        drawPulleyJoint = drawPulleyJoint,
        drawRevoluteJoint = drawRevoluteJoint,
        drawRopeJoint = drawRopeJoint,
        drawWeldJoint = drawWeldJoint,
        -- presenter hooks
        presentBody = presentBody,
        presentFixture = presentFixture,
        presentJoint = presentJoint,
        -- api
        setAngle = setAngle,
        getAngle = getAngle,
        setCamera = setCamera,
        getCamera = getCamera,
        setViewport = setViewport,
        getViewport = getViewport,
        setScale = setScale,
        getScale = getScale,
        draw = draw,
        screenToWorld = screenToWorld,
        worldToScreen = worldToScreen,
        setPresenter = setPresenter,
        getPresenter = getPresenter,
    }
end
