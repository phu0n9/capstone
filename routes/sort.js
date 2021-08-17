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

router.route('/sortByLocation').get(jwtCheck,async (req,res) => {
    await Inventory.find({'location':req.query.location}).sort({'createdAt':-1})
    .exec((err, inventory) => {
        if (err) return (err)
        res.send(inventory) // return data JSON
    })
})

router.route('/sortByUserId').get(jwtCheck,async (req,res) => {
    await Inventory.find({'userId':req.query.userId}).sort({'createdAt':-1})
    .exec((err, inventory) => {
        if (err) return err
        res.send(inventory) // return data JSON
    })
})

router.route('/sortByTime').get(jwtCheck,async (req,res) => {
    await Inventory.find({'createdAt':
        {
            '$gte' :new Date(req.query.time),
            // "$lt": new Date(req.query.time)
        }
    }).sort({'createdAt':-1})
    .exec((err, inventory) => {
        if (err) return err
        res.send(inventory) // return data JSON
    })
})

module.exports = router