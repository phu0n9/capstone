const router = require('express').Router()
const Inventory = require('../model/inventory.model')
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

router.route('').get(jwtCheck,async (req,res,next) => {
    const page = req.query.page || 1 // Page 
    const resPerPage = 5 // results per page
    await Inventory.find().sort({'createdAt':-1}).skip(page - resPerPage).limit(resPerPage)
    .exec((err, inventory) => {
      if (err) return next(err)
      res.send(inventory) // return data JSON
    })
})

module.exports = router