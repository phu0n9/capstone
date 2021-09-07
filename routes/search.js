const router = require('express').Router()
const Queue = require('../model/queue.model')
const jwt = require('express-jwt')
const jwks = require('jwks-rsa')

const jwtCheck = jwt({
    secret: jwks.expressJwtSecret({
        cache: true,
        rateLimit: true,
        jwksRequestsPerMinute: 5,
        jwksUri: process.env.JWKS_URI
  }),
  audience: process.env.JWT_AUDIENCE,
  issuer: process.env.JWT_ISSUER,
  algorithms: ['RS256']
})

router.route('').post(jwtCheck,async (req, res)=>{
    await Queue.create({keyword:req.body.keyword,userId:req.body.userId})
    .then(() => res.send("success added to queue"))
    .catch((error)=>{console.log(error)})
})


module.exports = router

