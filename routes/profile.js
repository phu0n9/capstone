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

router.route('/date').get(jwtCheck,async (req, res)=>{
  const userId = req.query.userId
  await Inventory.aggregate([
    { 
        $match: { 
            'userId': userId,
            $expr: { 
                $gt: [ { $toDate: "$_id" },  { $toDate: new Date(new Date().toISOString() - 1000*60*60*24*365) } ],
            }
        }
    },
    { 
        $group: { 
            _id: { date: { 
              $dateFromParts : {
                  day: { $dayOfMonth: "$_id" },
                  month: { $month: "$_id" }, 
                  year: { $year: "$_id" }
              }
          } 
        }, 
          count: { $sum: 1 } 
        } 
    },
    { 
        $sort: { "_id.date" : -1 } 
    },
    { 
        $project: { 
            _id: 0, 
            count: 1, 
            date: { $dateToString: { date: "$_id.date", format: "%Y-%m-%d" } } 
        } 
    }
  ])
  .then((inventory)=>{res.status(200).send(inventory)})
  .catch((error)=>{res.status(401).send(error)})

})

module.exports = router
